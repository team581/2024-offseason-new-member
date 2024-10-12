package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.RobotConfig;
import frc.robot.controller.RumbleControllerSubsystem;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.snaps.SnapManager;
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
  private final IntakeSubsystem intake = new IntakeSubsystem(hd.intakeMotor);
  private final QueuerSubsystem queuer = new QueuerSubsystem(hd.queuerMotor, hd.sensor);
  private final SwerveSubsystem swerve = new SwerveSubsystem(hd.driverController);
  private final ImuSubsystem imu = new ImuSubsystem(swerve);
  private final FmsSubsystem fms = new FmsSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem(imu);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(swerve, imu, vision);
  private final SnapManager snaps = new SnapManager(swerve, hd.driverController);
  private final RobotManager robotManager =
      new RobotManager(intake, queuer, shooter, climber, localization, vision, swerve, snaps, imu);
  private final RobotCommands actions = new RobotCommands(robotManager);

  private Command autonomousCommand;

  private final Hardware hardware = new Hardware();

  public Robot() {
    System.out.println("roboRIO serial number: " + RobotConfig.SERIAL_NUMBER);

    DogLog.setOptions(
        new DogLogOptions().withCaptureNt(false).withNtPublish(RobotConfig.IS_DEVELOPMENT));
    DogLog.setPdh(hardware.pdh);

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
    autonomousCommand = Commands.none();

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
    autonomousCommand = autos.getAutoCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
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
    swerve.setDefaultCommand(swerve.driveTeleopCommand());

    // Driver controller
    hd.driverController.back().onTrue(localization.getZeroCommand());

    hd.driverController.y().onTrue(snaps.getCommand(() -> SnapManager.getSourceAngle()));
    hd.driverController.x().onTrue(snaps.getCommand(() -> SnapManager.getStageLeftAngle()));
    hd.driverController.b().onTrue(snaps.getCommand(() -> SnapManager.getStageRightAngle()));
    hd.driverController.a().onTrue(snaps.getCommand(() -> SnapManager.getAmpAngle()));
    hd.driverController.povUp().onTrue(snaps.getCommand(() -> SnapManager.getStageBackChain()));

    hd.driverController
        .leftTrigger()
        .onTrue(actions.intakeCommand())
        .onFalse(actions.idleCommand());
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
    hd.operatorController.rightBumper().onTrue(actions.climbSequenceCommand());
    hd.operatorController.leftBumper().onTrue(actions.reverseClimbSequenceCommand());
    hd.operatorController.povLeft().onTrue(actions.unjamCommand()).onFalse(actions.idleCommand());
    hd.operatorController.back().onTrue(actions.homeClimberCommand());
  }
}
