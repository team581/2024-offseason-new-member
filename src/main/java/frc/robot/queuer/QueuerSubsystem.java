package frc.robot.queuer;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class QueuerSubsystem extends StateMachine<QueuerState> {
  private final TalonFX motor;
  private final DigitalInput sensor;

  public QueuerSubsystem(TalonFX motor, DigitalInput sensor) {
    super(SubsystemPriority.QUEUER, QueuerState.IDLE);

    motor.getConfigurator().apply(RobotConfig.get().queuer().motorConfig());

    this.sensor = sensor;
    this.motor = motor;
  }

  @Override
  protected void afterTransition(QueuerState newstate) {
    switch (newstate) {
      case IDLE -> {
        motor.setVoltage(0);
      }
      case TO_INTAKE -> {
        motor.setVoltage(-1);
      }
      case TO_SHOOTER -> {
        motor.setVoltage(12);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Queuer/HasNote", hasNote());
  }

  public boolean hasNote() {
    return sensor.get();
  }

  public void setState(QueuerState state) {
    setStateFromRequest(state);
  }

  @Override
  protected QueuerState getNextState(QueuerState state) {
    return state;
  }
}
