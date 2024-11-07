package frc.robot.config;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.PerfToggles;
import frc.robot.config.RobotConfig.ShooterConfig;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;
import frc.robot.vision.interpolation.InterpolatedVisionDataset;

class CompConfig {
  private static final String RIO_CAN_NAME = "rio";

  private static final ClosedLoopRampsConfigs CLOSED_LOOP_RAMP =
      new ClosedLoopRampsConfigs()
          .withDutyCycleClosedLoopRampPeriod(0.04)
          .withTorqueClosedLoopRampPeriod(0.04)
          .withVoltageClosedLoopRampPeriod(0.04);
  private static final OpenLoopRampsConfigs OPEN_LOOP_RAMP =
      new OpenLoopRampsConfigs()
          .withDutyCycleOpenLoopRampPeriod(0.04)
          .withTorqueOpenLoopRampPeriod(0.04)
          .withVoltageOpenLoopRampPeriod(0.04);

  // TODO: change gear ratios
  public static final RobotConfig competitionBot =
      new RobotConfig(
          "competition",
          new IntakeConfig(
              4,
              0,
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(50)
                          .withSupplyCurrentLimitEnable(true))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)),
          new ShooterConfig(
              2,
              3,
              70,
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(80)
                          .withSupplyCurrentLimitEnable(true))
                  .withTorqueCurrent(
                      new TorqueCurrentConfigs()
                          .withPeakForwardTorqueCurrent(200)
                          .withPeakReverseTorqueCurrent(0))
                  .withSlot0(new Slot0Configs().withKV(0.15).withKP(3.5).withKI(0).withKD(0))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.CounterClockwise_Positive))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP),
              new TalonFXConfiguration()
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(80)
                          .withSupplyCurrentLimitEnable(true))
                  .withTorqueCurrent(
                      new TorqueCurrentConfigs()
                          .withPeakForwardTorqueCurrent(200)
                          .withPeakReverseTorqueCurrent(0))
                  .withSlot0(new Slot0Configs().withKV(0.15).withKP(2).withKI(0).withKD(0))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.CounterClockwise_Positive))
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP),
              topFlywheelDistanceToRPM -> {
                topFlywheelDistanceToRPM.put(0.0, 0.0);
              },
              bottomFlywheelDistanceToRPM -> {
                bottomFlywheelDistanceToRPM.put(0.0, 0.0);
              }),
          new ClimberConfig(20, 1.5, 11.0, 0.0, 0.25),
          new SwerveConfig(
              // new PhoenixPIDController(50, 0, 5),
              new PhoenixPIDController(4, 0, 0), // 20 0 2
              true,
              true,
              true,
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimit(80)
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true),
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(80)
                  .withPeakReverseTorqueCurrent(-80)),
          new VisionConfig(
              4,
              0.4,
              0.4,
              tyToNoteDistance -> {
                tyToNoteDistance.put(-19.9, Units.inchesToMeters(17.75 + 7 - 1.5));
                tyToNoteDistance.put(-14.815, Units.inchesToMeters(17.75 + 7 + 3.75));
                tyToNoteDistance.put(-6.3, Units.inchesToMeters(17.75 + 7 + 14.0));
                tyToNoteDistance.put(0.4, Units.inchesToMeters(17.75 + 7 + 22.9));
                tyToNoteDistance.put(5.65, Units.inchesToMeters(17.75 + 7 + 34.25));
                tyToNoteDistance.put(9.39, Units.inchesToMeters(17.75 + 7 + 47.1));
                tyToNoteDistance.put(11.85, Units.inchesToMeters(17.75 + 7 + 60.1));
                tyToNoteDistance.put(15.25, Units.inchesToMeters(17.75 + 7 + 88.9));
              },
              InterpolatedVisionDataset.CHEZY_CHAMPS),
          new PerfToggles(true, false, false));

  private CompConfig() {}
}
