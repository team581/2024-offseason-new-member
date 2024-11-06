package frc.robot.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {
  private static final double ENCODER_CONVERSION_FACTOR =
      (1.0 / 64.0) * RobotConfig.get().climber().axleDiameter() * (Math.PI);

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private double height = 0;
  private double encoderPosition = 0;
  private boolean manualDownMode = false;

  public ClimberSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.CLIMBER, ClimberState.LOWERED);

    this.encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(1);

    motor.setSmartCurrentLimit(50);
    motor.burnFlash();

    this.motor = motor;

    encoder.setPosition(0);
  }

  @Override
  protected void collectInputs() {
    encoderPosition = -1.0 * encoder.getPosition();
    height = encoderPosition * ENCODER_CONVERSION_FACTOR;
  }

  public void setManualDownMode(boolean manualDownMode) {
    this.manualDownMode = manualDownMode;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Climber/Height", height);
    DogLog.log("Climber/EncoderPosition", encoderPosition);
    DogLog.log("Climber/StatorCurrent", motor.getOutputCurrent());
    DogLog.log("Climber/OutputVoltage", motor.getAppliedOutput());
    DogLog.log("Climber/ManualDownMode", manualDownMode);

    if (manualDownMode) {
      // Run down at a low voltage in manual down mode
      motor.set(-0.2);
      return;
    }

    var goalHeight =
        switch (getState()) {
          case RAISED -> RobotConfig.get().climber().maxHeight();
          case LOWERED -> RobotConfig.get().climber().minHeight();
        };

    if (MathUtil.isNear(goalHeight, height, RobotConfig.get().climber().toleranceInches())) {
      // At goal
      motor.disable();
    } else if (height < goalHeight) {
      motor.set(-0.3);
    } else {
      motor.set(0.3);
    }
  }

  public void setState(ClimberState state) {
    setStateFromRequest(state);
  }

  @Override
  protected ClimberState getNextState(ClimberState state) {
    return state;
  }
}
