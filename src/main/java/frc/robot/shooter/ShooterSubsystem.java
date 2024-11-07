package frc.robot.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ShooterSubsystem extends StateMachine<ShooterState> {
  private final TalonFX bottomMotor;
  private final TalonFX topMotor;
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0).withLimitReverseMotion(true);
  private final InterpolatingDoubleTreeMap speakerToRpmTop = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap speakerToRpmBottom = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feedingToRpmTop = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feedingToRpmBottom = new InterpolatingDoubleTreeMap();
  private double speakerDistance = 0.0;
  private double feedDistance = 0;
  private double bottomMotorRpm = 0;
  private double topMotorRpm = 0;

  public ShooterSubsystem(TalonFX bottomMotor, TalonFX topMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);

    topMotor.getConfigurator().apply(RobotConfig.get().shooter().topMotorConfig());
    bottomMotor.getConfigurator().apply(RobotConfig.get().shooter().bottomMotorConfig());

    RobotConfig.get().shooter().topFlywheelSpeakerDistanceToRpm().accept(speakerToRpmTop);
    RobotConfig.get().shooter().bottomFlywheelSpeakerDistanceToRpm().accept(speakerToRpmBottom);

    RobotConfig.get().shooter().topFlywheelFeedingDistanceToRpm().accept(feedingToRpmTop);
    RobotConfig.get().shooter().bottomFlywheelFeedingDistanceToRpm().accept(feedingToRpmBottom);

    this.bottomMotor = bottomMotor;
    this.topMotor = topMotor;
  }

  @Override
  protected void collectInputs() {
    topMotorRpm = topMotor.getVelocity().getValueAsDouble() * 60.0;
    bottomMotorRpm = bottomMotor.getVelocity().getValueAsDouble() * 60.0;
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
            velocityRequest.withVelocity(speakerToRpmTop.get(speakerDistance) / 60.0).withSlot(0));
        bottomMotor.setControl(
            velocityRequest
                .withVelocity(speakerToRpmBottom.get(speakerDistance) / 60.0)
                .withSlot(0));
      }
      case FLOOR_SHOT -> {
        topMotor.setControl(
            velocityRequest.withVelocity(speakerToRpmTop.get(feedDistance) / 60.0).withSlot(0));
        bottomMotor.setControl(
            velocityRequest.withVelocity(speakerToRpmBottom.get(feedDistance) / 60.0).withSlot(0));
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
    DogLog.log("ShooterSubsystem/TopMotor/RPM", topMotorRpm);
    DogLog.log("ShooterSubsystem/TopMotor/GoalRPM", getState().topRPM);
    DogLog.log("ShooterSubsystem/BottomMotor/RPM", bottomMotorRpm);
    DogLog.log("ShooterSubsystem/BottomMotor/GoalRPM", getState().bottomRPM);
    DogLog.log("ShooterSubsystem/State", getState());
    DogLog.log("ShooterSubsystem/AtGoal", atGoal(getState()));
  }

  public boolean atGoal(ShooterState state) {
    if (state != getState()) {
      return false;
    }
    return switch (state) {
      case STOPPED -> true;
      case SPEAKER_SHOT -> {
        var topRpmGoal = speakerToRpmTop.get(speakerDistance);
        var bottomRpmGoal = speakerToRpmBottom.get(speakerDistance);
        yield MathUtil.isNear(topRpmGoal, topMotorRpm, RobotConfig.get().shooter().tolerance())
            && MathUtil.isNear(
                bottomRpmGoal, bottomMotorRpm, RobotConfig.get().shooter().tolerance());
      }
      case FLOOR_SHOT -> {
        var topRpmGoal = feedingToRpmTop.get(feedDistance);
        var bottomRpmGoal = feedingToRpmBottom.get(feedDistance);

        yield MathUtil.isNear(topRpmGoal, topMotorRpm, RobotConfig.get().shooter().tolerance())
            && MathUtil.isNear(
                bottomRpmGoal, bottomMotorRpm, RobotConfig.get().shooter().tolerance());
      }
      default -> {
        var topRpmGoal = getState().topRPM;
        var bottomRpmGoal = getState().bottomRPM;
        yield MathUtil.isNear(topRpmGoal, topMotorRpm, RobotConfig.get().shooter().tolerance())
            && MathUtil.isNear(
                bottomRpmGoal, bottomMotorRpm, RobotConfig.get().shooter().tolerance());
      }
    };
  }

  public void setSpeakerDistance(double distance) {
    speakerDistance = distance;
  }

  public void setFeedDistance(double distance) {
    feedDistance = distance;
  }

  public void setState(ShooterState newState) {
    setStateFromRequest(newState);
  }
}
