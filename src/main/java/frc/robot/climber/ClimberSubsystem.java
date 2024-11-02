package frc.robot.climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {
  ClimberConfig CONFIG = RobotConfig.get().climber();
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkPIDController pid;

  private LinearFilter currentFilter = LinearFilter.movingAverage(CONFIG.currentTaps());
  private double tolerance = 1.5; // cm

  public ClimberSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.CLIMBER, ClimberState.NOT_HOMED);

    this.encoder = motor.getEncoder();
    this.pid = motor.getPIDController();
    pid.setP(0);
    pid.setI(0);
    pid.setD(0);

    encoder.setPositionConversionFactor(2 * Math.PI);
    motor.burnFlash();

    this.motor = motor;
  }

  @Override
  protected void afterTransition(ClimberState newState) {

    double rawCurrent = motor.getOutputCurrent();
    double filteredCurrent = currentFilter.calculate(rawCurrent);

    switch (newState) {
      case NOT_HOMED -> {
        motor.disable();
      }

      case HOMING -> {
        motor.setVoltage(CONFIG.homingVoltage());
        if (filteredCurrent >= CONFIG.homingCurrentThreshold()) {
          encoder.setPosition(CONFIG.minHeight());
          setState(ClimberState.LOWERED);
        }
      }
      case RAISED -> {
        pid.setReference(CONFIG.maxHeight() / CONFIG.axleRadius(), ControlType.kPosition);
      }
      case LOWERED -> {
        pid.setReference(CONFIG.minHeight() / CONFIG.axleRadius(), ControlType.kPosition);
      }
    }
    DogLog.log("Climber/FilteredCurrent", filteredCurrent);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Climber/RotationRadians", encoder.getPosition());
    DogLog.log("Climber/Height", getHeight());
    DogLog.log("Climber/State", getState());
    DogLog.log("Climber/AtGoal", atGoal());
  }

  public boolean atGoal() {
    if (getState() == ClimberState.HOMING || getState() == ClimberState.NOT_HOMED) {
      return false;
    }

    if (getState() == ClimberState.RAISED) {
      return Math.abs(getHeight() - CONFIG.maxHeight()) <= tolerance;
    } else {
      return Math.abs(getHeight() - CONFIG.minHeight()) <= tolerance;
    }
  }

  public void setState(ClimberState state) {
    if (getState() == ClimberState.NOT_HOMED || getState() == ClimberState.HOMING) {
      return;
    }
    setStateFromRequest(state);
  }

  public double getHeight() {
    return encoder.getPosition() * CONFIG.axleRadius();
  }

  @Override
  protected ClimberState getNextState(ClimberState state) {
    return state;
  }
}
