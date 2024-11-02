package frc.robot.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import dev.doglog.DogLog;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.security.InvalidParameterException;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class SwerveSubsystem extends StateMachine<SwerveState> {
  /** Max speed allowed to make a speaker shot and feeding. */
  private static final double MAX_SPEED_SHOOTING = Units.feetToMeters(0.5);

  private static final double MAX_FLOOR_SPEED_SHOOTING = Units.feetToMeters(18);

  public static final double MaxSpeed = 4.75;
  private static final double MaxAngularRate = Units.rotationsToRadians(4);
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double leftXDeadband = 0.05;
  private static final double rightXDeadband = 0.05;
  private static final double leftYDeadband = 0.05;
  private final CommandXboxController controller;
  private boolean snapToAngle = false;
  boolean closedLoop = false;

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
  private final PIDController xPid = new PIDController(3.2, 0, 0);
  private final PIDController yPid = new PIDController(3.2, 0, 0);

  /** The latest requested teleop speeds. */
  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();

  private ChassisSpeeds autoSpeeds = new ChassisSpeeds();

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

  public double snapAngle() {
    return goalSnapAngle;
  }

  public SwerveSubsystem(CommandXboxController controller) {
    super(SubsystemPriority.SWERVE, SwerveState.TELEOP);
    modulePositions = calculateModulePositions();
    this.controller = controller;

    for (int i = 0; i < 4; i++) {
      var module = drivetrain.getModule(i);
      var driveMotorConfigurator = module.getDriveMotor().getConfigurator();

      driveMotorConfigurator.apply(RobotConfig.get().swerve().driveMotorCurrentLimits());
      driveMotorConfigurator.apply(RobotConfig.get().swerve().driveMotorTorqueCurrentLimits());
    }
  }

  public void setFieldRelativeAutoSpeeds(ChassisSpeeds speeds) {
    autoSpeeds = speeds;
    sendSwerveRequest();
  }

  public void disableSnaps() {
    snapToAngle = false;
  }

  public void enableSnaps() {
    snapToAngle = true;
  }

  public boolean snapToAngleEnabled() {
    return snapToAngle;
  }

  private static SwerveModuleConstants constantsForModuleNumber(int moduleNumber) {
    return switch (moduleNumber) {
      case 0 -> TunerConstants.FrontLeft;
      case 1 -> TunerConstants.FrontRight;
      case 2 -> TunerConstants.BackLeft;
      case 3 -> TunerConstants.BackRight;
      default -> throw new InvalidParameterException("Expected an ID from [0, 3]");
    };
  }

  public void driveTeleop() {
    double leftY =
        -1.0
            * ControllerHelpers.getExponent(
                ControllerHelpers.getDeadbanded(controller.getLeftY(), leftYDeadband), 1.5);
    double leftX =
        ControllerHelpers.getExponent(
            ControllerHelpers.getDeadbanded(controller.getLeftX(), leftXDeadband), 1.5);
    double rightX =
        ControllerHelpers.getExponent(
            ControllerHelpers.getDeadbanded(controller.getRightX(), rightXDeadband), 2);

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

  public Command driveTeleopCommand() {
    return run(() -> driveTeleop()).withName("DriveTeleopCommand");
  }

  @Override
  protected void collectInputs() {
    modulePositions = calculateModulePositions();
    robotRelativeSpeeds = calculateRobotRelativeSpeeds();
    fieldRelativeSpeeds = calculateFieldRelativeSpeeds();
    slowEnoughToShoot = calculateMovingSlowEnoughForSpeakerShot(robotRelativeSpeeds);
    slowEnoughToFeed = calculateMovingSlowEnoughForFloorShot(robotRelativeSpeeds);

   var moduleStates = drivetrain.getState().ModuleStates;

    // Sometimes we check for module states before they seem to be populated
    // This avoids a crash because of that
    if (moduleStates == null) {
      moduleStates  = new SwerveModuleState[] {};
    }

    DogLog.log("Swerve/ModuleStates", moduleStates);
  }

  private List<SwerveModulePosition> calculateModulePositions() {
    return List.of(
        frontLeft.getPosition(true),
        frontRight.getPosition(true),
        backLeft.getPosition(true),
        backRight.getPosition(true));
  }

  public void setFieldRelativeSpeeds(ChassisSpeeds speeds, boolean closedLoop) {
    this.fieldRelativeSpeeds = speeds;
    this.closedLoop = closedLoop;
    // Send a swerve request each time new chassis speeds are provided
    sendSwerveRequest();
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds speeds, boolean closedLoop) {
    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drivetrain.getState().Pose.getRotation());
    setFieldRelativeSpeeds(fieldRelative, closedLoop);
  }

  private ChassisSpeeds calculateRobotRelativeSpeeds() {
    var moduleStates = drivetrain.getState().ModuleStates;

    // Sometimes we check for module states before they seem to be populated
    // This avoids a crash because of that
    if (moduleStates == null) {
      return new ChassisSpeeds();
    }

    return KINEMATICS.toChassisSpeeds(moduleStates);
  }

  private ChassisSpeeds calculateFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds, Rotation2d.fromDegrees(drivetrainPigeon.getYaw().getValueAsDouble()));
  }

  public boolean movingSlowEnoughForSpeakerShot() {
    return calculateMovingSlowEnoughForSpeakerShot(getRobotRelativeSpeeds());
  }

  public boolean movingSlowEnoughForFloorShot() {
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    double linearSpeed =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    return linearSpeed < MAX_FLOOR_SPEED_SHOOTING;
  }

  public boolean calculateMovingSlowEnoughForSpeakerShot(ChassisSpeeds speeds) {
    double linearSpeed =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    return linearSpeed < MAX_SPEED_SHOOTING;
  }

  private boolean calculateMovingSlowEnoughForFloorShot(ChassisSpeeds speeds) {
    double linearSpeed =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    return linearSpeed < MAX_FLOOR_SPEED_SHOOTING;
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
        if (snapToAngle) {
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
    }
  }

  private boolean atLocation(Pose2d target, Pose2d current) {
    // Return true once at location
    double translationTolerance = 0.1;
    double omegaTolerance = 5;
    return Math.abs(current.getX() - target.getX()) <= translationTolerance
        && Math.abs(current.getY() - target.getY()) <= translationTolerance
        && Math.abs(current.getRotation().getDegrees() - target.getRotation().getDegrees())
            <= omegaTolerance;
  }

  public Command driveToPoseCommand(
      Supplier<Optional<Pose2d>> targetSupplier, Supplier<Pose2d> currentPose, boolean shouldEnd) {
    return run(() -> {
          var maybeTarget = targetSupplier.get();

          if (!maybeTarget.isPresent()) {
            setFieldRelativeSpeeds(new ChassisSpeeds(), closedLoop);
            return;
          }
          var target = maybeTarget.get();

          var pose = currentPose.get();
          double vx = xPid.calculate(pose.getX(), target.getX());
          double vy = yPid.calculate(pose.getY(), target.getY());

          var newSpeeds = new ChassisSpeeds(vx, vy, 0);
          setFieldRelativeSpeeds(newSpeeds, true);
        })
        .until(
            () -> {
              if (shouldEnd) {
                var maybeTarget = targetSupplier.get();
                if (maybeTarget.isPresent()) {
                  var target = targetSupplier.get();

                  return atLocation(target.get(), currentPose.get());
                }
              }

              return false;
            })
        .finallyDo(
            () -> {
              snapToAngle = false;
            });
  }
}
