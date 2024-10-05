package frc.robot.climber;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class ClimberSubsystem {
  ClimberConfig CONFIG = RobotConfig.get().climber();
  private final TalonFX motor;
  private double goalDistance = 0.0;
  private PositionVoltage positionRequest = new PositionVoltage(goalDistance);
  private LinearFilter currentFilter = LinearFilter.movingAverage(CONFIG.currentTaps());
  private boolean raised = false;
  private boolean homing = false;
  private double tolerance = 1.5; // cm

  public ClimberSubsystem(TalonFX motor) {
    super(SubsystemPriority.CLIMBER);

    motor.getConfigurator().apply(RobotConfig.get().climber().motorConfig());

    this.motor = motor;
  }

  @Override
  public void robotPeriodic() {
    double rawCurrent = motor.getStatorCurrent().getValueAsDouble();
    double filteredCurrent = currentFilter.calculate(rawCurrent);

    if (homing == true) {
      motor.setVoltage(CONFIG.homingVoltage());
      if (filteredCurrent >= CONFIG.homingCurrentThreshold()) {
        motor.setPosition(0.0);
        homing = false;
      }
    } else if (homing == false) {
      if (raised) {
        motor.setControl(positionRequest.withPosition(CONFIG.maxHeight()));
      } else {
        motor.setControl(positionRequest.withPosition(0.0));
      }
    }

    DogLog.log("Climber/Height", getHeight());
    DogLog.log("Climber/FilteredCurrent", filteredCurrent);
    DogLog.log("Climber/Homing", homing);
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
    // TODO: tune axle radius
    var axleRadius = 1;
    return motor.getPosition().getValueAsDouble() * 2 * Math.PI * axleRadius;
  }

  public void unhomed() {
    homing = false;
  }

  public void startHoming() {
    homing = true;
  }

  public void setRaised(boolean raise) {
    raised = raise;
  }

  public boolean getRaised() {
    return raised;
  }
}
