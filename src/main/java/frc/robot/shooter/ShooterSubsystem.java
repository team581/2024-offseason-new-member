package frc.robot.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ShooterSubsystem extends StateMachine<ShooterState> {
  private final TalonFX bottomMotor;
  private final TalonFX topMotor;
  private final double tolerance = RobotConfig.get().shooter().tolerance();
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0).withLimitReverseMotion(true);

  public ShooterSubsystem(TalonFX bottomMotor, TalonFX topMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);

    topMotor.getConfigurator().apply(RobotConfig.get().shooter().topMotorConfig());
    bottomMotor.getConfigurator().apply(RobotConfig.get().shooter().bottomMotorConfig());

    this.bottomMotor = bottomMotor;
    this.topMotor = topMotor;
  }

  @Override
  protected void afterTransition(ShooterState newState) {
    switch (newState) {
      case STOPPED -> {
        topMotor.disable();
        bottomMotor.disable();
      }
      default -> {
        topMotor.setControl(velocityRequest.withVelocity(getState().RPM));
        bottomMotor.setControl(velocityRequest.withVelocity(getState().RPM));
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Shooter/TopMotor/RPM", getTopMotorRPM());
    DogLog.log("Shooter/BottomMotor/RPM", getBottomMotorRPM());
    DogLog.log("Shooter/GoalState", getState());
    DogLog.log("Shooter/GoalRPM", getState().RPM);
    DogLog.log("Shooter/AtGoal", atGoal(getState()));
  }

  public boolean atGoal(ShooterState state) {
    if (state != getState()) {
      return false;
    }

    if (getState() == ShooterState.STOPPED) {
      return true;
    }

    if (Math.abs(getState().RPM - getBottomMotorRPM()) <= tolerance
        && Math.abs(getState().RPM - getTopMotorRPM()) <= tolerance) {
      return true;
    }

    return false;
  }

  @Override
  protected ShooterState getNextState(ShooterState state) {
    return state;
  }

  public void setState(ShooterState newState) {
    setStateFromRequest(newState);
  }

  public double getBottomMotorRPM() {
    return bottomMotor.getVelocity().getValueAsDouble() * 60;
  }

  public double getTopMotorRPM() {
    return topMotor.getVelocity().getValueAsDouble() * 60;
  }
}
