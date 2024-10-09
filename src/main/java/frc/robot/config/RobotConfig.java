package frc.robot.config;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.vision.interpolation.InterpolatedVisionDataset;
import java.util.function.Consumer;

public record RobotConfig(
    String robotName,
    String canivoreName,
    IntakeConfig intake,
    QueuerConfig queuer,
    ShooterConfig shooter,
    ClimberConfig climber,
    IMUConfig imu,
    SwerveConfig swerve,
    VisionConfig vision,
    PerfToggles perfToggles) {

  public record IntakeConfig(int motorID, TalonFXConfiguration motorConfig) {}

  public record QueuerConfig(int motorID, int sensorID, TalonFXConfiguration motorConfig) {}

  public record ShooterConfig(
      int bottomMotorID,
      int topMotorID,
      double tolerance,
      TalonFXConfiguration bottomMotorConfig,
      TalonFXConfiguration topMotorConfig) {}

  public record ClimberConfig(
      int motorID,
      double maxHeight,
      double homingVoltage,
      double homingCurrentThreshold,
      int currentTaps,
      TalonFXConfiguration motorConfig) {}

  public record VisionConfig(
      int translationHistoryArraySize,
      double xyStdDev,
      double thetaStdDev,
      Consumer<InterpolatingDoubleTreeMap> tyToNoteDistance,
      InterpolatedVisionDataset interpolatedVisionSet) {}

  public record PerfToggles(
      boolean interpolatedVision, boolean noteMapInTeleop, boolean noteMapBoundingBox) {}

  public record IMUConfig(
      int deviceID, Consumer<InterpolatingDoubleTreeMap> distanceToAngleTolerance) {}

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
  private static final String PRACTICE_BOT_SERIAL_NUMBER = "0322443D"; // TODO: this is fake
  public static final boolean IS_PRACTICE_BOT =
      SERIAL_NUMBER != null && SERIAL_NUMBER.equals(PRACTICE_BOT_SERIAL_NUMBER);

  public static RobotConfig get() {
    return CompConfig.competitionBot;
  }
}
