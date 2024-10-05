package frc.robot.config;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

public record RobotConfig(
    String robotName,
    String canivoreName,
    IntakeConfig intake,
    QueuerConfig queuer,
    ShooterConfig shooter,
    ClimberConfig climber,
    SwerveConfig swerve) {

  public record IntakeConfig(
      int motorID, double intakeVoltage, double outtakeVoltage, TalonFXConfiguration motorConfig) {}

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
