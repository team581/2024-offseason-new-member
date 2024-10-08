package frc.robot.util.scheduling;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class LifecycleSubsystemManager {
  private static LifecycleSubsystemManager instance;

  public static LifecycleSubsystemManager getInstance() {
    if (instance == null) {
      instance = new LifecycleSubsystemManager();
    }

    return instance;
  }

  public static LifecycleStage getStage() {
    if (DriverStation.isTeleopEnabled()) {
      return LifecycleStage.TELEOP;
    } else if (DriverStation.isAutonomousEnabled()) {
      return LifecycleStage.AUTONOMOUS;
    } else if (DriverStation.isTestEnabled()) {
      return LifecycleStage.TEST;
    } else {

      return LifecycleStage.DISABLED;
    }
  }

  private final List<LifecycleSubsystem> subsystems = new ArrayList<>();
  private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

  private LifecycleSubsystemManager() {}

  public void ready() {
    Collections.sort(
        subsystems,
        Comparator.comparingInt((LifecycleSubsystem subsystem) -> subsystem.priority.value)
            .reversed());

    for (LifecycleSubsystem lifecycleSubsystem : subsystems) {
      commandScheduler.registerSubsystem(lifecycleSubsystem);
    }
  }

  public void log() {
    DogLog.log("Scheduler/Stage", getStage().toString());
  }

  void registerSubsystem(LifecycleSubsystem subsystem) {
    subsystems.add(subsystem);
    commandScheduler.unregisterSubsystem(subsystem);
  }
}
