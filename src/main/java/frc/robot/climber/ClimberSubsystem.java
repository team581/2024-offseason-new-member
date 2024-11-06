package frc.robot.climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import dev.doglog.DogLog;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkPIDController pid;

  private static final double ENCODER_CONVERSION_FACTOR = (1.0/64.0) * RobotConfig.get().climber().axleDiameter() * (Math.PI);

  public ClimberSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.CLIMBER, ClimberState.LOWERED);

    this.encoder = motor.getEncoder();
    this.pid = motor.getPIDController();
    pid.setP(100.0);
    pid.setI(0);
    pid.setD(0);

    motor.setSmartCurrentLimit(100);
    motor.burnFlash();

    this.motor = motor;

    encoder.setPosition(0);
  }

  @Override
  protected void afterTransition(ClimberState newState) {
    switch (newState) {
      case RAISED -> {
        pid.setReference(RobotConfig.get().climber().maxHeight() / ENCODER_CONVERSION_FACTOR, ControlType.kPosition);
      }
      case LOWERED -> {
        pid.setReference(RobotConfig.get().climber().minHeight() / ENCODER_CONVERSION_FACTOR, ControlType.kPosition);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("ClimberSubsystem/Height", getHeight());
    DogLog.log("ClimberSubsystem/State", getState());
    DogLog.log("ClimberSubsystem/StatorCurrent", motor.getOutputCurrent());
    DogLog.log("ClimberSubsystem/OutputVoltage",     motor.getAppliedOutput());
  }

  public void setState(ClimberState state) {
    setStateFromRequest(state);
  }

  public double getHeight() {
    return encoder.getPosition() * ENCODER_CONVERSION_FACTOR;
  }

  @Override
  protected ClimberState getNextState(ClimberState state) {
    return state;
  }
}
