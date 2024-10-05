package frc.robot.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class ShooterSubsystem extends LifecycleSubsystem {
  private final TalonFX bottomMotor;
  private final TalonFX topMotor;
  private final double tolerance = RobotConfig.get().shooter().tolerance();
  private ShooterState goalState = ShooterState.IDLE;
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0).withLimitReverseMotion(true);

  public ShooterSubsystem(TalonFX bottomMotor, TalonFX topMotor) {
    super(SubsystemPriority.SHOOTER);

    topMotor.getConfigurator().apply(RobotConfig.get().shooter().topMotorConfig());
    bottomMotor.getConfigurator().apply(RobotConfig.get().shooter().bottomMotorConfig());

    this.bottomMotor = bottomMotor;
    this.topMotor = topMotor;
  }

  @Override
  public void robotPeriodic() {
    if (goalState == ShooterState.STOPPED) {
      topMotor.disable();
      bottomMotor.disable();
    } else {
      topMotor.setControl(velocityRequest.withVelocity(goalState.RPM));
      bottomMotor.setControl(velocityRequest.withVelocity(goalState.RPM));
    }
    DogLog.log("Shooter/TopMotor/RPM", getTopMotorRPM());
    DogLog.log("Shooter/BottomMotor/RPM", getBottomMotorRPM());
    DogLog.log("Shooter/GoalState", goalState);
    DogLog.log("Shooter/GoalRPM", goalState.RPM);
    DogLog.log("Shooter/AtGoal", atGoal(goalState));
  }

  public boolean atGoal(ShooterState state) {
    if (state != goalState) {
      return false;
    }

    if (goalState == ShooterState.STOPPED) {
      return true;
    }

    if (Math.abs(goalState.RPM - getBottomMotorRPM()) <= tolerance
        && Math.abs(goalState.RPM - getTopMotorRPM()) <= tolerance) {
      return true;
    }

    return false;
  }

  public void setState(ShooterState newState) {
    goalState = newState;
  }

  public ShooterState getState() {
    return goalState;
  }

  public double getBottomMotorRPM() {
    return bottomMotor.getVelocity().getValueAsDouble() * 60;
  }

  public double getTopMotorRPM() {
    return topMotor.getVelocity().getValueAsDouble() * 60;
  }
}
