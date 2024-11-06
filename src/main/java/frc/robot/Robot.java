package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.Autos;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.RobotConfig;
import frc.robot.controller.RumbleControllerSubsystem;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.Stopwatch;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.vision.VisionSubsystem;

public class Robot extends TimedRobot {
  private final Hardware hd = new Hardware();

  private final RumbleControllerSubsystem driverRumble =
      new RumbleControllerSubsystem(hd.driverController, false);
  private final RumbleControllerSubsystem operatorRumble =
      new RumbleControllerSubsystem(hd.operatorController, true);

  private final ClimberSubsystem climber = new ClimberSubsystem(hd.climberMotor);
  private final ShooterSubsystem shooter = new ShooterSubsystem(hd.bottomShooter, hd.topShooter);
  private final IntakeSubsystem intake = new IntakeSubsystem(hd.intakeMotor, hd.sensor);
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ImuSubsystem imu = new ImuSubsystem(swerve);
  private final FmsSubsystem fms = new FmsSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem(imu);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(swerve, imu, vision);
  private final RobotManager robotManager =
      new RobotManager(intake, shooter, climber, localization, vision, swerve, imu);
  private final RobotCommands actions = new RobotCommands(robotManager);
  private final Autos autos = new Autos(swerve, localization, actions, robotManager);

  private Command autonomousCommand;

  public Robot() {
    System.out.println("roboRIO serial number: " + RobotConfig.SERIAL_NUMBER);

    DogLog.setOptions(
        new DogLogOptions().withCaptureNt(false).withNtPublish(RobotConfig.IS_DEVELOPMENT));
    DogLog.setPdh(hd.pdp);

    // Record metadata
    DogLog.log("Metadata/ProjectName", BuildConstants.MAVEN_NAME);
    DogLog.log("Metadata/RoborioSerialNumber", RobotConfig.SERIAL_NUMBER);
    DogLog.log("Metadata/RobotName", RobotConfig.get().robotName());
    DogLog.log("Metadata/BuildDate", BuildConstants.BUILD_DATE);
    DogLog.log("Metadata/GitSHA", BuildConstants.GIT_SHA);
    DogLog.log("Metadata/GitDate", BuildConstants.GIT_DATE);
    DogLog.log("Metadata/GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        DogLog.log("Metadata/GitDirty", "All changes committed");
        break;
      case 1:
        DogLog.log("Metadata/GitDirty", "Uncomitted changes");
        break;
      default:
        DogLog.log("Metadata/GitDirty", "Unknown");
        break;
    }

    // This must be run before any commands are scheduled
    LifecycleSubsystemManager.getInstance().ready();

    SmartDashboard.putData(CommandScheduler.getInstance());

    configureBindings();
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    Stopwatch.getInstance().start("Scheduler/CommandSchedulerPeriodic");
    CommandScheduler.getInstance().run();
    Stopwatch.getInstance().stop("Scheduler/CommandSchedulerPeriodic");
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = autos.getAutoCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
      ;
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.run(
            () -> {
              if (DriverStation.isTeleop()) {
                swerve.driveTeleop(
                    hd.driverController.getLeftX(),
                    hd.driverController.getLeftY(),
                    hd.driverController.getRightX());
              }
            }));

    // Driver controller
    hd.driverController.back().onTrue(localization.getZeroCommand());

    hd.driverController
        .leftTrigger()
        .onTrue(
            actions
                .intakeCommand()
                .alongWith(
                    robotManager
                        .waitForState(RobotState.INTAKING)
                        .andThen(Commands.waitUntil(() -> robotManager.getState().hasNote))
                        .andThen(driverRumble.getRumbleShortCommand())))
        .onFalse(
            actions
                .idleCommand()
                .alongWith(Commands.waitUntil(() -> robotManager.getState().hasNote))
                .andThen(driverRumble.getRumbleShortCommand()));
    hd.driverController
        .rightTrigger()
        .onTrue(actions.confirmShotCommand())
        .onFalse(actions.idleCommand());
    hd.driverController
        .rightBumper()
        .onTrue(actions.outtakeCommand())
        .onFalse(actions.idleCommand());

    // operator controller

    hd.operatorController
        .leftTrigger()
        .onTrue(actions.waitFloorCommand())
        .onFalse(actions.idleCommand());
    hd.operatorController
        .rightTrigger()
        .onTrue(actions.waitSpeakerCommand())
        .onFalse(actions.idleCommand());
    hd.operatorController.a().onTrue(actions.waitSubwooferCommand()).onFalse(actions.idleCommand());
    hd.operatorController.x().onTrue(actions.waitAmpCommand()).onFalse(actions.idleCommand());

    hd.operatorController.povUp().onTrue(actions.nextClimbCommand());
    hd.operatorController.povDown().onTrue(actions.reverseClimbCommand());
    hd.operatorController.povLeft().onTrue(actions.unjamCommand()).onFalse(actions.idleCommand());

    // Press start + back button together for manual down mode
    hd.operatorController
        .start()
        .and(hd.operatorController.back())
        .onTrue(Commands.runOnce(() -> climber.setManualDownMode(true)))
        .onFalse(Commands.runOnce(() -> climber.setManualDownMode(false)));
  }
}
