package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.filter.Debouncer;

public record RobotConfig(String robotName, String canivoreName, IntakeConfig intake) {

  public record IntakeConfig(
      int motorID, double intakeVoltage, double outtakeVoltage, TalonFXConfiguration motorConfig) {}

  public record QueuerConfig(
      int motorID, int sensorID, Debouncer debouncer, TalonFXConfiguration motorConfig) {}

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  public static RobotConfig get() {
    return CompConfig.competitionBot;
  }
}
