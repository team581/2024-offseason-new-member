package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public record RobotConfig(String robotName, String canivoreName, IntakeSubsystem intake) {

  public record IntakeSubsystem(
      int motorID, double intakeVoltage, double outtakeVoltage, TalonFXConfiguration motorConfig) {}

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  public static RobotConfig get() {
    return CompConfig.competitionBot;
  }
}
