package frc.robot.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.intake_assist.IntakeAssistManager;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.util.List;

public class SwerveSubsystem extends StateMachine<SwerveState> {
  /** Max speed allowed to make a speaker shot. */
  private static final double MAX_SPEED_SHOOTING = Units.feetToMeters(0.5);

  /** Max speed allowed for feeding. */
  private static final double MAX_FEED_SPEED_SHOOTING = Units.feetToMeters(0.5);

  public static final double MaxSpeed = 4.75;
  private static final double MaxAngularRate = Units.rotationsToRadians(4);
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double leftXDeadband = 0.05;
  private static final double rightXDeadband = 0.07;
  private static final double leftYDeadband = 0.05;

  public static final Translation2d FRONT_LEFT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.FrontLeft.LocationX, CommandSwerveDrivetrain.FrontLeft.LocationY);
  public static final Translation2d FRONT_RIGHT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.FrontRight.LocationX,
          CommandSwerveDrivetrain.FrontRight.LocationY);
  public static final Translation2d BACK_LEFT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.BackLeft.LocationX, CommandSwerveDrivetrain.BackLeft.LocationY);
  public static final Translation2d BACK_RIGHT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.BackRight.LocationX, CommandSwerveDrivetrain.BackRight.LocationY);
  public static final Translation2d[] MODULE_LOCATIONS =
      new Translation2d[] {
        FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION
      };
  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(MODULE_LOCATIONS);

  private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain();

  public final Pigeon2 drivetrainPigeon = drivetrain.getPigeon2();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          // I want field-centric driving in open loop
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03)
          .withRotationalDeadband(MaxAngularRate * 0.03);

  private final SwerveRequest.FieldCentricFacingAngle driveToAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03);
  private final SwerveModule frontLeft = drivetrain.getModule(0);
  private final SwerveModule frontRight = drivetrain.getModule(1);
  private final SwerveModule backLeft = drivetrain.getModule(2);
  private final SwerveModule backRight = drivetrain.getModule(3);

  private boolean slowEnoughToShoot = false;
  private List<SwerveModulePosition> modulePositions;
  private ChassisSpeeds robotRelativeSpeeds;
  private ChassisSpeeds fieldRelativeSpeeds;
  private boolean slowEnoughToFeed;
  private double goalSnapAngle = 0;

  /** The latest requested teleop speeds. */
  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();

  private ChassisSpeeds autoSpeeds = new ChassisSpeeds();

  private ChassisSpeeds intakeAssistTeleopSpeeds = new ChassisSpeeds();

  private ChassisSpeeds intakeAssistAutoSpeeds = new ChassisSpeeds();

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  public boolean isSlowEnoughToShoot() {
    return slowEnoughToShoot;
  }

  public boolean isSlowEnoughToFeed() {
    return slowEnoughToFeed;
  }

  public List<SwerveModulePosition> getModulePositions() {
    return modulePositions;
  }

  public void setSnapToAngle(double angle) {
    goalSnapAngle = angle;
  }

  public SwerveSubsystem() {
    super(SubsystemPriority.SWERVE, SwerveState.TELEOP);
    driveToAngle.HeadingController = RobotConfig.get().swerve().snapController();
    driveToAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveToAngle.HeadingController.setTolerance(Math.toRadians(1.5));
    modulePositions = calculateModulePositions();

    for (int i = 0; i < 4; i++) {
      var module = drivetrain.getModule(i);
      var driveMotorConfigurator = module.getDriveMotor().getConfigurator();

      driveMotorConfigurator.apply(RobotConfig.get().swerve().driveMotorCurrentLimits());
      driveMotorConfigurator.apply(RobotConfig.get().swerve().driveMotorTorqueCurrentLimits());
    }
  }

  public double getGoalSnapAngle() {
    return goalSnapAngle;
  }

  @Override
  protected SwerveState getNextState(SwerveState currentState) {
    // Ensure that we are in an auto state during auto, and a teleop state during teleop
    return switch (currentState) {
      case AUTO, TELEOP -> DriverStation.isAutonomous() ? SwerveState.AUTO : SwerveState.TELEOP;
      case INTAKE_ASSIST_AUTO, INTAKE_ASSIST_TELEOP ->
          DriverStation.isAutonomous()
              ? SwerveState.INTAKE_ASSIST_AUTO
              : SwerveState.INTAKE_ASSIST_TELEOP;
      case AUTO_SNAPS, TELEOP_SNAPS ->
          DriverStation.isAutonomous() ? SwerveState.AUTO_SNAPS : SwerveState.TELEOP_SNAPS;
    };
  }

  public void setRobotRelativeAutoSpeeds(ChassisSpeeds speeds) {
    setFieldRelativeAutoSpeeds(
        ChassisSpeeds.fromRobotRelativeSpeeds(
            speeds, Rotation2d.fromDegrees(drivetrainPigeon.getYaw().getValueAsDouble())));
  }

  public void setFieldRelativeAutoSpeeds(ChassisSpeeds speeds) {
    autoSpeeds = speeds;
    sendSwerveRequest();
  }

  public void driveTeleop(double x, double y, double theta) {
    double leftY =
        -1.0
            * ControllerHelpers.getExponent(ControllerHelpers.getDeadbanded(y, leftYDeadband), 1.5);
    double leftX =
        ControllerHelpers.getExponent(ControllerHelpers.getDeadbanded(x, leftXDeadband), 1.5);
    double rightX =
        ControllerHelpers.getExponent(ControllerHelpers.getDeadbanded(theta, rightXDeadband), 2);

    if (RobotConfig.get().swerve().invertRotation()) {
      rightX *= -1.0;
    }

    if (RobotConfig.get().swerve().invertX()) {
      leftX *= -1.0;
    }

    if (RobotConfig.get().swerve().invertY()) {
      leftY *= -1.0;
    }

    if (FmsSubsystem.isRedAlliance()) {
      leftX *= -1.0;
      leftY *= -1.0;
    }

    teleopSpeeds =
        new ChassisSpeeds(
            -1.0 * leftY * MaxSpeed,
            leftX * MaxSpeed,
            rightX * TELEOP_MAX_ANGULAR_RATE.getRadians());

    sendSwerveRequest();
  }

  @Override
  protected void collectInputs() {
    modulePositions = calculateModulePositions();
    robotRelativeSpeeds = calculateRobotRelativeSpeeds();
    fieldRelativeSpeeds = calculateFieldRelativeSpeeds();
    slowEnoughToShoot = calculateMovingSlowEnoughForSpeakerShot(robotRelativeSpeeds);
    slowEnoughToFeed = calculateMovingSlowEnoughForFeed(robotRelativeSpeeds);
  }

  private List<SwerveModulePosition> calculateModulePositions() {
    return List.of(
        frontLeft.getPosition(true),
        frontRight.getPosition(true),
        backLeft.getPosition(true),
        backRight.getPosition(true));
  }

  private ChassisSpeeds calculateRobotRelativeSpeeds() {
    return KINEMATICS.toChassisSpeeds(drivetrain.getState().ModuleStates);
  }

  private ChassisSpeeds calculateFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds, Rotation2d.fromDegrees(drivetrainPigeon.getYaw().getValueAsDouble()));
  }

  private static boolean calculateMovingSlowEnoughForSpeakerShot(ChassisSpeeds speeds) {
    double linearSpeed =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    return linearSpeed < MAX_SPEED_SHOOTING;
  }

  private boolean calculateMovingSlowEnoughForFeed(ChassisSpeeds speeds) {
    double linearSpeed =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    return linearSpeed < MAX_FEED_SPEED_SHOOTING;
  }

  private void sendSwerveRequest() {
    switch (getState()) {
      case TELEOP ->
          drivetrain.setControl(
              drive
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond)
                  .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      case TELEOP_SNAPS -> {
        if (teleopSpeeds.omegaRadiansPerSecond == 0) {
          drivetrain.setControl(
              driveToAngle
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond)
                  .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        } else {
          drivetrain.setControl(
              drive
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond)
                  .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }
      }
      case INTAKE_ASSIST_TELEOP -> {
        intakeAssistTeleopSpeeds =
            IntakeAssistManager.getRobotRelativeAssistSpeeds(0, teleopSpeeds);

        drivetrain.setControl(
            drive
                .withVelocityX(intakeAssistTeleopSpeeds.vxMetersPerSecond)
                .withVelocityY(intakeAssistTeleopSpeeds.vyMetersPerSecond)
                .withRotationalRate(intakeAssistTeleopSpeeds.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      }
      case AUTO ->
          drivetrain.setControl(
              drive
                  .withVelocityX(autoSpeeds.vxMetersPerSecond)
                  .withVelocityY(autoSpeeds.vyMetersPerSecond)
                  .withRotationalRate(autoSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.Velocity));
      case AUTO_SNAPS ->
          drivetrain.setControl(
              driveToAngle
                  .withVelocityX(autoSpeeds.vxMetersPerSecond)
                  .withVelocityY(autoSpeeds.vyMetersPerSecond)
                  .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                  .withDriveRequestType(DriveRequestType.Velocity));

      case INTAKE_ASSIST_AUTO -> {
        intakeAssistAutoSpeeds = IntakeAssistManager.getRobotRelativeAssistSpeeds(0, autoSpeeds);

        drivetrain.setControl(
            drive
                .withVelocityX(intakeAssistAutoSpeeds.vxMetersPerSecond)
                .withVelocityY(intakeAssistAutoSpeeds.vyMetersPerSecond)
                .withRotationalRate(intakeAssistAutoSpeeds.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.Velocity));
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Swerve/GoalSnapAngle", goalSnapAngle);
  }

  public void setState(SwerveState newState) {
    setStateFromRequest(newState);
  }

  public void setSnapsEnabled(boolean newValue) {
    switch (getState()) {
      case TELEOP, TELEOP_SNAPS, INTAKE_ASSIST_TELEOP ->
          setStateFromRequest(newValue ? SwerveState.TELEOP_SNAPS : SwerveState.TELEOP);
      case AUTO, AUTO_SNAPS, INTAKE_ASSIST_AUTO ->
          setStateFromRequest(newValue ? SwerveState.AUTO_SNAPS : SwerveState.AUTO);
    }
  }
}
