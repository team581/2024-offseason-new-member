package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {
  private final TalonFX motor;
  IntakeConfig CONFIG = RobotConfig.get().intake();

  public IntakeSubsystem(TalonFX motor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);

    motor.getConfigurator().apply(CONFIG.motorConfig());

    this.motor = motor;
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE -> {
        motor.disable();
      }
      default -> {
        motor.setVoltage(getState().volts);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
  }

  public void setState(IntakeState state) {
    setStateFromRequest(state);
  }

  @Override
  protected IntakeState getNextState(IntakeState currentState) {
    return currentState;
  }
}
