package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {
  private final TalonFX motor;
  private final DigitalInput sensor;
  private boolean rawHasNote = false;
  private boolean debouncedHasNote = false;

  IntakeConfig CONFIG = RobotConfig.get().intake();

  public IntakeSubsystem(TalonFX motor, DigitalInput sensor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);

    motor.getConfigurator().apply(CONFIG.motorConfig());

    this.sensor = sensor;

    this.motor = motor;
  }

  @Override
  protected void collectInputs() {
    rawHasNote = sensor.get();
    debouncedHasNote = CONFIG.debouncer().calculate(rawHasNote);
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

    DogLog.log("IntakeSubsystem/MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("IntakeSubsystem/State", getState());
    DogLog.log("IntakeSubsystem/DebouncedHasNote", debouncedHasNote);
    DogLog.log("IntakeSubsystem/RawHasNote", rawHasNote);
    DogLog.log("IntakeSubsystem/State/Volts", getState().volts);
  }

  public void setState(IntakeState state) {
    setStateFromRequest(state);
  }

  public boolean hasNote() {
    return debouncedHasNote;
  }

  @Override
  protected IntakeState getNextState(IntakeState currentState) {
    return currentState;
  }
}
