package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import dev.doglog.DogLog;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {
  private final TalonFX motor;
  private final CANSparkMax funnelMotor;
  IntakeConfig CONFIG = RobotConfig.get().intake();

  public IntakeSubsystem(TalonFX motor, CANSparkMax funnelMotor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);

    motor.getConfigurator().apply(CONFIG.motorConfig());

    this.motor = motor;
    this.funnelMotor = funnelMotor;
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE -> {
        motor.disable();
        funnelMotor.disable();
      }
      default -> {
        motor.setVoltage(getState().volts);
        funnelMotor.setVoltage(getState().volts * (getState().inverted ? -1 : 1));
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("IntakeSubsystem/MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("IntakeSubsystem/FunnelMotorVoltage", funnelMotor.getAppliedOutput());
  }

  public void setState(IntakeState state) {
    setStateFromRequest(state);
  }

  @Override
  protected IntakeState getNextState(IntakeState currentState) {
    return currentState;
  }
}
