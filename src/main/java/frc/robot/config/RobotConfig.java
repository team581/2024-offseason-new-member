package frc.robot.config;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.vision.interpolation.InterpolatedVisionDataset;
import java.util.function.Consumer;

public record RobotConfig(
    String robotName,
    IntakeConfig intake,
    ShooterConfig shooter,
    ClimberConfig climber,
    SwerveConfig swerve,
    VisionConfig vision,
    PerfToggles perfToggles) {

  public record IntakeConfig(
      int motorID, int sensorID, Debouncer debouncer, TalonFXConfiguration motorConfig) {}

  public record ShooterConfig(
      int bottomMotorID,
      int topMotorID,
      double tolerance,
      TalonFXConfiguration bottomMotorConfig,
      TalonFXConfiguration topMotorConfig,
      Consumer<InterpolatingDoubleTreeMap> topFlywheelSpeakerDistanceToRpm,
      Consumer<InterpolatingDoubleTreeMap> bottomFlywheelSpeakerDistanceToRpm,
      Consumer<InterpolatingDoubleTreeMap> topFlywheelFeedingDistanceToRpm,
      Consumer<InterpolatingDoubleTreeMap> bottomFlywheelFeedingDistanceToRpm) {}

  public record ClimberConfig(
      int motorID,
      double axleDiameter,
      double maxHeight,
      double minHeight,
      double toleranceInches) {}

  public record VisionConfig(
      int translationHistoryArraySize,
      double xyStdDev,
      double thetaStdDev,
      Consumer<InterpolatingDoubleTreeMap> tyToNoteDistance,
      InterpolatedVisionDataset interpolatedVisionSet) {}

  public record PerfToggles(
      boolean interpolatedVision, boolean noteMapInTeleop, boolean noteMapBoundingBox) {}

  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY,
      CurrentLimitsConfigs driveMotorCurrentLimits,
      TorqueCurrentConfigs driveMotorTorqueCurrentLimits) {}

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  public static RobotConfig get() {
    return CompConfig.competitionBot;
  }
}
