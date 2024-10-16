package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;

public class Hardware {
  private RobotConfig CONFIG = RobotConfig.get();
  public final PowerDistribution pdh = new PowerDistribution();

  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  public final CANSparkMax climberMotor = new CANSparkMax(CONFIG.climber().motorID(), MotorType.kBrushless);
  public final TalonFX bottomShooter = new TalonFX(CONFIG.shooter().bottomMotorID());
  public final TalonFX topShooter = new TalonFX(CONFIG.shooter().topMotorID());
  public final TalonFX intakeMotor = new TalonFX(CONFIG.intake().motorID());
  public final TalonFX queuerMotor = new TalonFX(CONFIG.queuer().motorID());
  public final DigitalInput sensor = new DigitalInput(CONFIG.queuer().sensorID());
}
