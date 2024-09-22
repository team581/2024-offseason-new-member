// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.queuer;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class QueuerSubsystem extends LifecycleSubsystem {

  private final TalonFX motor;
  private final DigitalInput sensor;
  private QueuerState goalState = QueuerState.IDLE;

  public QueuerSubsystem(TalonFX motor, DigitalInput sensor) {
    super(SubsystemPriority.QUEUER);

    motor.getConfigurator().apply(RobotConfig.get().queuer().motorConfig());

    this.sensor = sensor;
    this.motor = motor;
  }

  @Override
  public void robotPeriodic() {
    DogLog.log("Queuer/State", goalState);
    DogLog.log("Queuer/HasNote", hasNote());

    switch (goalState) {
      case IDLE:
        motor.disable();
        break;
      case TO_INTAKE:
        motor.setVoltage(-1);
        break;
      case TO_SHOOTER:
        motor.setVoltage(12);
        break;
      default:
        break;
    }
  }
  
    public boolean hasNote() {
      return sensor.get();
    }

  public void setState(QueuerState state) {
    goalState = state;
  }

  public QueuerState getState() {
    return goalState;
  }
}
