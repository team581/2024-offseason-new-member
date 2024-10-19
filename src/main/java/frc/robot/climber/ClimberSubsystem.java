package frc.robot.climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<HomingState> {
  ClimberConfig CONFIG = RobotConfig.get().climber();
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkPIDController pid;

  private double goalDistance = 0.0;
  private LinearFilter currentFilter = LinearFilter.movingAverage(CONFIG.currentTaps());
  private boolean raised = false;
  private double tolerance = 1.5; // cm
  // TODO: tune axle radius
  private double axleRadius = 1;

  public ClimberSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.CLIMBER, HomingState.NOT_HOMED);

    this.encoder = motor.getEncoder();
    this.pid = motor.getPIDController();
    // TODO: tune pid and make more config for motor
    pid.setP(1.0);
    pid.setI(1.0);
    pid.setD(1.0);

    this.motor = motor;
  }

  @Override
  protected void afterTransition(HomingState newState) {

    double rawCurrent = motor.getOutputCurrent();
    double filteredCurrent = currentFilter.calculate(rawCurrent);

    switch (newState) {
      case NOT_HOMED -> {}

      case MID_MATCH_HOMING -> {
        motor.setVoltage(CONFIG.homingVoltage());
        if (filteredCurrent >= CONFIG.homingCurrentThreshold()) {
          encoder.setPosition(0.0);
          setState(HomingState.HOMED);
        }
      }

      case PRE_MATCH_HOMING -> {
        motor.setVoltage(CONFIG.homingVoltage());
        if (filteredCurrent >= CONFIG.homingCurrentThreshold()) {
          encoder.setPosition(0.0);
          setState(HomingState.HOMED);
        }
      }

      case HOMED -> {
        if (raised) {
          pid.setReference(CONFIG.maxHeight() / (2 * Math.PI * axleRadius), ControlType.kPosition);
        } else {
          pid.setReference(0.0, ControlType.kPosition);
        }
      }
    }
    DogLog.log("Climber/FilteredCurrent", filteredCurrent);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Climber/Height", getHeight());
    DogLog.log("Climber/Raised", getRaised());
    DogLog.log("Climber/AtGoal", atGoal());
  }

  public boolean atGoal() {
    if (raised == true) {
      return Math.abs(getHeight() - CONFIG.maxHeight()) <= tolerance;
    } else {
      return Math.abs(getHeight()) <= tolerance;
    }
  }

  public double getHeight() {
    return encoder.getPosition() * 2 * Math.PI * axleRadius;
  }

  public void setState(HomingState state) {
    setStateFromRequest(state);
  }

  @Override
  protected HomingState getNextState(HomingState state) {
    return state;
  }

  public void setRaised(boolean bool) {
    raised = bool;
  }

  public boolean getRaised() {
    return raised;
  }
}
