package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import frc.robot.generated.BuildConstants;
import frc.robot.util.Stopwatch;
import frc.robot.util.scheduling.LifecycleSubsystemManager;

public class Robot extends TimedRobot {
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
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
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

  private void configureBindings() {}
}
