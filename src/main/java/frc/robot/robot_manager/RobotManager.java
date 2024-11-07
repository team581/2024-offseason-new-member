package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.climber.ClimberState;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.shooter.ShooterState;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveState;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.DistanceAngle;
import frc.robot.vision.VisionState;
import frc.robot.vision.VisionSubsystem;

public class RobotManager extends StateMachine<RobotState> {
  public final IntakeSubsystem intake;
  public final ShooterSubsystem shooter;
  public final ClimberSubsystem climber;
  public final LocalizationSubsystem localization;
  public final VisionSubsystem vision;
  public final SwerveSubsystem swerve;
  public final ImuSubsystem imu;

  // INIT
  public RobotManager(
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      ClimberSubsystem climber,
      LocalizationSubsystem localization,
      VisionSubsystem vision,
      SwerveSubsystem swerve,
      ImuSubsystem imu) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
    this.intake = intake;
    this.shooter = shooter;
    this.climber = climber;
    this.localization = localization;
    this.vision = vision;
    this.swerve = swerve;
    this.imu = imu;
  }

  // --------------------
  DistanceAngle speakerDistanceAngle = new DistanceAngle(0, 0, false);
  DistanceAngle floorDistanceAngle = new DistanceAngle(0, 0, false);
  boolean angularVelocitySlowEnough;
  boolean robotHeadingAtGoal;
  boolean limelightWorking;
  boolean swerveSlowEnough;

  @Override
  protected void collectInputs() {
    speakerDistanceAngle = vision.getDistanceAngleSpeaker();
    floorDistanceAngle = vision.getDistanceAngleFloorShot();

    double feedDistance = floorDistanceAngle.distance();
    double speakerDistance = speakerDistanceAngle.distance();

    DogLog.log("Debug/FeedDistance", feedDistance);

    switch (getState()) {
      case PREPARE_FLOOR_SHOT, WAITING_FLOOR_SHOT, FLOOR_SHOT -> {
        robotHeadingAtGoal = imu.atAngleForFeed(floorDistanceAngle.targetAngle());
        swerveSlowEnough = swerve.isSlowEnoughToFeed();
        angularVelocitySlowEnough = Math.abs(imu.getRobotAngularVelocity()) < 360.0;
      }
      case PREPARE_SPEAKER_SHOT, WAITING_SPEAKER_SHOT, SPEAKER_SHOT -> {
        robotHeadingAtGoal =
            imu.atAngleForSpeaker(speakerDistanceAngle.targetAngle(), speakerDistance);
        angularVelocitySlowEnough = imu.belowVelocityForVision(speakerDistance);
        swerveSlowEnough = swerve.isSlowEnoughToShoot();
      }

      default -> {
        robotHeadingAtGoal =
            imu.atAngleForSpeaker(speakerDistanceAngle.targetAngle(), speakerDistance);
      }
    }

    limelightWorking = false;
    if (DriverStation.isAutonomous() && vision.getState() == VisionState.OFFLINE) {
      limelightWorking = true;
    } else {
      limelightWorking = vision.getState() == VisionState.SEES_TAGS;
    }

    shooter.setSpeakerDistance(speakerDistance);
    shooter.setFeedDistance(feedDistance);
  }

  // Automatic state transitions
  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case WAITING_SUBWOOFER_SHOT,
              WAITING_FLOOR_SHOT,
              WAITING_SPEAKER_SHOT,
              WAITING_AMP,
              IDLE_NO_GP,
              IDLE_W_GP,
              CLIMB_LOWERED,
              CLIMB_RAISED,
              UNJAM,
              OUTTAKING,
              WAITING_SHOOTER_OUTTAKE ->
          currentState;
      case SUBWOOFER_SHOT, FLOOR_SHOT, SPEAKER_SHOT, SHOOTER_OUTTAKE, AMP_SHOT ->
          intake.hasNote() ? currentState : RobotState.IDLE_NO_GP;

      case PREPARE_SUBWOOFER_SHOT ->
          shooter.atGoal(ShooterState.SUBWOOFER_SHOT) ? RobotState.SUBWOOFER_SHOT : currentState;

      case PREPARE_AMP -> shooter.atGoal(ShooterState.AMP) ? RobotState.AMP_SHOT : currentState;

      case PREPARE_SPEAKER_SHOT ->
          (shooter.atGoal(ShooterState.SPEAKER_SHOT)
                  && swerveSlowEnough
                  && angularVelocitySlowEnough
                  && robotHeadingAtGoal
                  && limelightWorking)
              ? RobotState.SPEAKER_SHOT
              : currentState;

      case PREPARE_FLOOR_SHOT ->
          shooter.atGoal(ShooterState.FLOOR_SHOT)
                  && swerveSlowEnough
                  && robotHeadingAtGoal
                  && angularVelocitySlowEnough
              ? RobotState.FLOOR_SHOT
              : currentState;

      case PREPARE_SHOOTER_OUTTAKE ->
          shooter.atGoal(ShooterState.SHOOTER_OUTTAKE) ? RobotState.SHOOTER_OUTTAKE : currentState;

      case INTAKING -> intake.hasNote() ? RobotState.IDLE_W_GP : currentState;
    };
  }

  // ---------------------

  // State actions
  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE_NO_GP -> {
        shooter.setState(ShooterState.STOPPED);
        intake.setState(IntakeState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }

      case IDLE_W_GP -> {
        shooter.setState(ShooterState.IDLE);
        intake.setState(IntakeState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
      case INTAKING -> {
        shooter.setState(ShooterState.STOPPED);
        intake.setState(IntakeState.INTAKE);
        if (DriverStation.isAutonomous()) {
          swerve.setState(SwerveState.INTAKE_ASSIST_AUTO);
        } else {
          swerve.setState(SwerveState.INTAKE_ASSIST_TELEOP);
        }
      }
      case OUTTAKING -> {
        shooter.setState(ShooterState.IDLE);
        intake.setState(IntakeState.OUTTAKE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
      case PREPARE_AMP -> {
        shooter.setState(ShooterState.AMP);
        intake.setState(IntakeState.IDLE);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getAmpAngle());
      }
      case SHOOTER_OUTTAKE -> {
        shooter.setState(ShooterState.SHOOTER_OUTTAKE);
        intake.setState(IntakeState.TO_SHOOTER);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
      case PREPARE_SHOOTER_OUTTAKE, WAITING_SHOOTER_OUTTAKE -> {
        shooter.setState(ShooterState.SHOOTER_OUTTAKE);
        intake.setState(IntakeState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
      case PREPARE_SUBWOOFER_SHOT, WAITING_SUBWOOFER_SHOT -> {
        shooter.setState(ShooterState.SUBWOOFER_SHOT);
        intake.setState(IntakeState.IDLE);
      }
      case PREPARE_SPEAKER_SHOT, WAITING_SPEAKER_SHOT -> {
        shooter.setState(ShooterState.SPEAKER_SHOT);
        intake.setState(IntakeState.IDLE);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(speakerDistanceAngle.targetAngle());
      }
      case PREPARE_FLOOR_SHOT, WAITING_FLOOR_SHOT -> {
        shooter.setState(ShooterState.FLOOR_SHOT);
        intake.setState(IntakeState.IDLE);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(floorDistanceAngle.targetAngle());
      }
      case FLOOR_SHOT -> {
        shooter.setState(ShooterState.FLOOR_SHOT);
        intake.setState(IntakeState.TO_SHOOTER);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(floorDistanceAngle.targetAngle());
      }
      case SPEAKER_SHOT -> {
        shooter.setState(ShooterState.SPEAKER_SHOT);
        intake.setState(IntakeState.TO_SHOOTER);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(speakerDistanceAngle.targetAngle());
      }
      case SUBWOOFER_SHOT -> {
        shooter.setState(ShooterState.SUBWOOFER_SHOT);
        intake.setState(IntakeState.TO_SHOOTER);
      }
      case WAITING_AMP -> {
        shooter.setState(ShooterState.AMP);
        intake.setState(IntakeState.IDLE);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getAmpAngle());
      }
      case AMP_SHOT -> {
        shooter.setState(ShooterState.AMP);
        intake.setState(IntakeState.TO_SHOOTER);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getAmpAngle());
      }
      case UNJAM -> {
        shooter.setState(ShooterState.SUBWOOFER_SHOT);
        intake.setState(IntakeState.TO_SHOOTER);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
      case CLIMB_LOWERED, CLIMB_RAISED -> {
        shooter.setState(ShooterState.STOPPED);
        intake.setState(IntakeState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
    }
    climber.setState(newState.climberRaised ? ClimberState.RAISED : ClimberState.LOWERED);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    switch (getState()) {
      case PREPARE_SPEAKER_SHOT, SPEAKER_SHOT, WAITING_SPEAKER_SHOT -> {
        swerve.setSnapToAngle(speakerDistanceAngle.targetAngle());
      }
      case PREPARE_FLOOR_SHOT, FLOOR_SHOT, WAITING_FLOOR_SHOT -> {
        swerve.setSnapToAngle(floorDistanceAngle.targetAngle());
      }
      default -> {}
    }

    DogLog.log("RobotManager/State", getState());
    DogLog.log("RobotManager/State/ClimberRaised", getState().climberRaised);
    DogLog.log("RobotManager/State/hasNote", getState().hasNote);
    DogLog.log("RobotManager/LimelightWorking", limelightWorking);
    DogLog.log("RobotManager/SwerveSlowEnough", swerveSlowEnough);
    DogLog.log("RobotManager/RobotHeadingAtGoal", robotHeadingAtGoal);
    DogLog.log("RobotManager/AngularVelocitySlowEnough", angularVelocitySlowEnough);
  }

  public void waitAmpRequest() {
    setStateFromRequest(RobotState.WAITING_AMP);
  }

  public void waitSubwooferShotRequest() {
    setStateFromRequest(RobotState.WAITING_SUBWOOFER_SHOT);
  }

  public void waitFloorShotRequest() {
    setStateFromRequest(RobotState.WAITING_FLOOR_SHOT);
  }

  public void speakerShotRequest() {
    setStateFromRequest(RobotState.PREPARE_SPEAKER_SHOT);
  }

  public void waitSpeakerShotRequest() {
    setStateFromRequest(RobotState.WAITING_SPEAKER_SHOT);
  }

  public void confirmShotRequest() {
    switch (getState()) {
      case CLIMB_LOWERED, CLIMB_RAISED -> {}

      case WAITING_SUBWOOFER_SHOT -> setStateFromRequest(RobotState.PREPARE_SUBWOOFER_SHOT);
      case WAITING_FLOOR_SHOT -> setStateFromRequest(RobotState.PREPARE_FLOOR_SHOT);
      case WAITING_SPEAKER_SHOT -> setStateFromRequest(RobotState.PREPARE_SPEAKER_SHOT);
      case WAITING_SHOOTER_OUTTAKE -> setStateFromRequest(RobotState.PREPARE_SHOOTER_OUTTAKE);
      case WAITING_AMP -> setStateFromRequest(RobotState.PREPARE_AMP);

      default -> {
        setStateFromRequest(RobotState.PREPARE_SPEAKER_SHOT);
      }
    }
  }

  public void unjamRequest() {
    setStateFromRequest(RobotState.UNJAM);
  }

  public void intakeRequest() {
    setStateFromRequest(RobotState.INTAKING);
  }

  public void outtakeRequest() {
    setStateFromRequest(RobotState.OUTTAKING);
  }

  public void outtakeShooterRequest() {
    setStateFromRequest(RobotState.PREPARE_SHOOTER_OUTTAKE);
  }

  public void idleRequest() {
    if (getState().hasNote) {
      setStateFromRequest(RobotState.IDLE_W_GP);
    } else {
      setStateFromRequest(RobotState.IDLE_NO_GP);
    }
  }

  public void nextClimbRequest() {
    switch (getState()) {
      default -> setStateFromRequest(RobotState.CLIMB_RAISED);
      case CLIMB_RAISED -> setStateFromRequest(RobotState.CLIMB_LOWERED);
      case CLIMB_LOWERED -> {}
    }
  }

  public void reverseClimbRequest() {
    switch (getState()) {
      case CLIMB_LOWERED -> setStateFromRequest(RobotState.CLIMB_RAISED);
      case CLIMB_RAISED -> idleRequest();
      default -> {}
    }
  }

  public void preloadNoteRequest() {
    setStateFromRequest(RobotState.IDLE_W_GP);
  }
}
