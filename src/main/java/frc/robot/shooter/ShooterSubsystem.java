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
  protected void afterTransition(ShooterState newState) {
    switch (newState) {
      case STOPPED -> {
        topMotor.disable();
        bottomMotor.disable();
      }
      case SPEAKER_SHOT -> {
        topMotor.setControl(
            velocityRequest.withVelocity(distanceToRPMTop.get(speakerDistance)).withSlot(0));
        bottomMotor.setControl(
            velocityRequest.withVelocity(distanceToRPMBottom.get(speakerDistance)).withSlot(0));
      }
      default -> {
        topMotor.setControl(velocityRequest.withVelocity(getState().topRPM / 60.0).withSlot(0));
        bottomMotor.setControl(
            velocityRequest.withVelocity(getState().bottomRPM / 60.0).withSlot(0));
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("ShooterSubsystem/TopMotor/RPM", getTopMotorRPM());
    DogLog.log("ShooterSubsystem/TopMotor/GoalRPM", getState().topRPM);
    DogLog.log("ShooterSubsystem/BottomMotor/RPM", getBottomMotorRPM());
    DogLog.log("ShooterSubsystem/BottomMotor/GoalRPM", getState().bottomRPM);
    DogLog.log("ShooterSubsystem/State", getState());
    DogLog.log("ShooterSubsystem/AtGoal", atGoal(getState()));
  }

  public boolean atGoal(ShooterState state) {
    if (state != getState()) {
      return false;
    }

    if (getState() == ShooterState.STOPPED) {
      return true;
    }

    if (Math.abs(getState().bottomRPM - getBottomMotorRPM()) <= tolerance
        && Math.abs(getState().topRPM - getTopMotorRPM()) <= tolerance) {
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
