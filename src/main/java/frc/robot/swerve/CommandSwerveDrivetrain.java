package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.config.RobotConfig;
import frc.robot.generated.CompBotTunerConstants;
import frc.robot.generated.PracticeBotTunerConstants;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  public static final SwerveModuleConstants BackRight =
      RobotConfig.IS_PRACTICE_BOT
          ? PracticeBotTunerConstants.BackRight
          : CompBotTunerConstants.BackRight;
  public static final SwerveModuleConstants BackLeft =
      RobotConfig.IS_PRACTICE_BOT
          ? PracticeBotTunerConstants.BackLeft
          : CompBotTunerConstants.BackLeft;
  public static final SwerveModuleConstants FrontRight =
      RobotConfig.IS_PRACTICE_BOT
          ? PracticeBotTunerConstants.FrontRight
          : CompBotTunerConstants.FrontRight;
  public static final SwerveModuleConstants FrontLeft =
      RobotConfig.IS_PRACTICE_BOT
          ? PracticeBotTunerConstants.FrontLeft
          : CompBotTunerConstants.FrontLeft;
  public static final SwerveDrivetrainConstants DrivetrainConstants =
      RobotConfig.IS_PRACTICE_BOT
          ? PracticeBotTunerConstants.DrivetrainConstants
          : CompBotTunerConstants.DrivetrainConstants;
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrivetrain() {
    this(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }
}
