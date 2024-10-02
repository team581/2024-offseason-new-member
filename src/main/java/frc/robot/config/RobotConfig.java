package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public record RobotConfig(String robotName, String canivoreName, IntakeConfig intake, QueuerConfig queuer, ShooterConfig shooter) {

  public record IntakeConfig(
      int motorID, double intakeVoltage, double outtakeVoltage, TalonFXConfiguration motorConfig) {}

  public record QueuerConfig(
      int motorID, int sensorID, TalonFXConfiguration motorConfig) {}

  public record ShooterConfig(
      int bottomMotorID, int topMotorID, double tolerance, TalonFXConfiguration bottomMotorConfig, TalonFXConfiguration topMotorConfig) {}

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  public static RobotConfig get() {
    return CompConfig.competitionBot;
  }
}
