package frc.robot.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ShooterConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ShooterSubsystem extends StateMachine<ShooterState> {
  private final TalonFX bottomMotor;
  private final TalonFX topMotor;
  private ShooterConfig CONFIG = RobotConfig.get().shooter();
  private final double tolerance = CONFIG.tolerance();
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0).withLimitReverseMotion(true);
  private double goalRPMBottom = 0.0;
  private double goalRPMTop = 0.0;
  private final InterpolatingDoubleTreeMap distanceToRPMTop = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap distanceToRPMBottom = new InterpolatingDoubleTreeMap();
  private double speakerDistance = 0.0;

  public ShooterSubsystem(TalonFX bottomMotor, TalonFX topMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);

    topMotor.getConfigurator().apply(CONFIG.topMotorConfig());
    bottomMotor.getConfigurator().apply(CONFIG.bottomMotorConfig());

    CONFIG.topFlywheelMap().accept(distanceToRPMTop);
    CONFIG.bottomFlywheelMap().accept(distanceToRPMBottom);

    this.bottomMotor = bottomMotor;
    this.topMotor = topMotor;
  }

  @Override
  protected ShooterState getNextState(ShooterState state) {
    return state;
  }

  @Override
  protected void collectInputs() {
    if (getState() == ShooterState.SPEAKER_SHOT) {
      goalRPMTop = distanceToRPMTop.get(speakerDistance);
      goalRPMBottom = distanceToRPMBottom.get(speakerDistance);
    } else {
      goalRPMBottom = getState().bottomRPM;
      goalRPMTop = getState().topRPM;
    }
  }

  @Override
  protected void afterTransition(ShooterState newState) {
    switch (newState) {
      case STOPPED -> {
        topMotor.disable();
        bottomMotor.disable();
      }
      default -> {
        topMotor.setControl(velocityRequest.withVelocity(goalRPMTop));
        bottomMotor.setControl(velocityRequest.withVelocity(goalRPMBottom));
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Shooter/TopMotor/RPM", getTopMotorRPM());
    DogLog.log("Shooter/TopMotor/GoalRPM", goalRPMTop);
    DogLog.log("Shooter/BottomMotor/RPM", getBottomMotorRPM());
    DogLog.log("Shooter/BottomMotor/GoalRPM", goalRPMBottom);
    DogLog.log("Shooter/State", getState());
    DogLog.log("Shooter/AtGoal", atGoal(getState()));
  }

  public boolean atGoal(ShooterState state) {
    if (state != getState()) {
      return false;
    }

    if (getState() == ShooterState.STOPPED) {
      return true;
    }

    if (Math.abs(goalRPMBottom - getBottomMotorRPM()) <= tolerance
        && Math.abs(goalRPMTop - getTopMotorRPM()) <= tolerance) {
      return true;
    }

    return false;
  }

  public void setSpeakerDistance(double distance) {
    speakerDistance = distance;
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
