// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class IntakeSubsystem extends LifecycleSubsystem {
  private final TalonFX motor;
  private IntakeState goalState = IntakeState.IDLE;

  public IntakeSubsystem(TalonFX motor) {
    super(SubsystemPriority.INTAKE);

    motor.getConfigurator().apply(RobotConfig.get().intake().motorConfig());

    this.motor = motor;
  }

  @Override
  public void robotPeriodic() {
    DogLog.log("Intake/State", goalState);

    switch (goalState) {
      case IDLE:
        motor.disable();
        break;
      case INTAKE:
        motor.setVoltage(RobotConfig.get().intake().intakeVoltage());
        break;
      case OUTTAKE:
        motor.setVoltage(RobotConfig.get().intake().outtakeVoltage());
        break;

      default:
        break;
    }
  }

  public void setState(IntakeState state) {
    goalState = state;
  }

  public IntakeState getState() {
    return goalState;
  }
}
