package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.climber.ClimberState;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.queuer.QueuerState;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.shooter.ShooterState;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.snaps.SnapManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.DistanceAngle;
import frc.robot.vision.VisionState;
import frc.robot.vision.VisionSubsystem;

public class RobotManager extends StateMachine<RobotState> {
  public final IntakeSubsystem intake;
  public final ShooterSubsystem shooter;
  public final QueuerSubsystem queuer;
  public final ClimberSubsystem climber;
  public final LocalizationSubsystem localization;
  public final VisionSubsystem vision;
  public final SwerveSubsystem swerve;
  public final SnapManager snaps;
  public final ImuSubsystem imu;

  // INIT
  public RobotManager(
      IntakeSubsystem intake,
      QueuerSubsystem queuer,
      ShooterSubsystem shooter,
      ClimberSubsystem climber,
      LocalizationSubsystem localization,
      VisionSubsystem vision,
      SwerveSubsystem swerve,
      SnapManager snaps,
      ImuSubsystem imu) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
    this.intake = intake;
    this.shooter = shooter;
    this.queuer = queuer;
    this.climber = climber;
    this.localization = localization;
    this.vision = vision;
    this.swerve = swerve;
    this.snaps = snaps;
    this.imu = imu;
  }

  // --------------------
  DistanceAngle speakerDistanceAngle;
  DistanceAngle floorDistanceAngle;
  boolean angularVelocitySlowEnough;
  boolean robotHeadingAtGoal;
  boolean limelightWorking;
  boolean swerveSlowEnough;

  @Override
  protected void collectInputs() {
    speakerDistanceAngle = vision.getDistanceAngleSpeaker();
    floorDistanceAngle = vision.getDistanceAngleFloorShot();
    double speakerDistance = speakerDistanceAngle.distance();

    switch (getState()) {
      case PREPARE_FLOOR_SHOT, WAITING_FLOOR_SHOT, FLOOR_SHOT -> {
        robotHeadingAtGoal = imu.atAngleForFloorSpot(speakerDistanceAngle.targetAngle());
        swerveSlowEnough = swerve.movingSlowEnoughForFloorShot();
        angularVelocitySlowEnough = Math.abs(imu.getRobotAngularVelocity()) < 360.0;
      }
      case PREPARE_SUBWOOFER_SHOT,
          WAITING_SUBWOOFER_SHOT,
          SUBWOOFER_SHOT,
          PREPARE_SPEAKER_SHOT,
          WAITING_SPEAKER_SHOT,
          SPEAKER_SHOT -> {
        robotHeadingAtGoal =
            imu.atAngleForSpeaker(speakerDistanceAngle.targetAngle(), speakerDistance);
        angularVelocitySlowEnough = imu.belowVelocityForVision(speakerDistance);
        swerveSlowEnough = swerve.movingSlowEnoughForSpeakerShot();
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
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("RobotManager/State", getState());
    DogLog.log("RobotManager/State/ClimberRaised", getState().climberRaised);
    DogLog.log("RobotManager/State/hasNote", getState().hasNote);
    DogLog.log("RobotManager/LimelightWorking", limelightWorking);
    DogLog.log("RobotManager/SwerveSlowEnough", swerveSlowEnough);
    DogLog.log("RobotManager/RobotHeadingAtGoal", robotHeadingAtGoal);
    DogLog.log("RobotManager/AngularVelocitySlowEnough", angularVelocitySlowEnough);
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
              WAITING_CLIMB,
              CLIMBED,
              UNJAM,
              WAITING_SHOOTER_OUTTAKE ->
          currentState;
      case SUBWOOFER_SHOT, FLOOR_SHOT, SPEAKER_SHOT, OUTTAKING, SHOOTER_OUTTAKE, AMP_SHOT ->
          queuer.hasNote() ? currentState : RobotState.IDLE_NO_GP;
      case CLIMBING -> climber.atGoal() ? RobotState.CLIMBED : currentState;

      case PREPARE_SUBWOOFER_SHOT ->
          shooter.atGoal(ShooterState.SUBWOOFER_SHOT) ? RobotState.SUBWOOFER_SHOT : currentState;

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

      case INTAKING -> queuer.hasNote() ? RobotState.IDLE_W_GP : currentState;
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
        queuer.setState(QueuerState.IDLE);
      }

      case IDLE_W_GP -> {
        shooter.setState(ShooterState.IDLE);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
      }
      case INTAKING -> {
        shooter.setState(ShooterState.STOPPED);
        intake.setState(IntakeState.INTAKE);
        queuer.setState(QueuerState.INTAKING);
      }
      case OUTTAKING -> {
        shooter.setState(ShooterState.IDLE);
        intake.setState(IntakeState.OUTTAKE);
        queuer.setState(QueuerState.TO_INTAKE);
      }
      case SHOOTER_OUTTAKE -> {
        shooter.setState(ShooterState.SHOOTER_OUTTAKE);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.TO_SHOOTER);
      }
      case PREPARE_SHOOTER_OUTTAKE, WAITING_SHOOTER_OUTTAKE -> {
        shooter.setState(ShooterState.SHOOTER_OUTTAKE);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
      }
      case PREPARE_SUBWOOFER_SHOT, WAITING_SUBWOOFER_SHOT -> {
        shooter.setState(ShooterState.SUBWOOFER_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
      }
      case PREPARE_SPEAKER_SHOT, WAITING_SPEAKER_SHOT -> {
        shooter.setState(ShooterState.SPEAKER_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);

        snaps.setAngle(speakerDistanceAngle.targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case PREPARE_FLOOR_SHOT, WAITING_FLOOR_SHOT -> {
        shooter.setState(ShooterState.FLOOR_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);

        snaps.setAngle(floorDistanceAngle.targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case FLOOR_SHOT -> {
        shooter.setState(ShooterState.FLOOR_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.TO_SHOOTER);

        snaps.setAngle(floorDistanceAngle.targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case SPEAKER_SHOT -> {
        shooter.setState(ShooterState.SPEAKER_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.TO_SHOOTER);

        snaps.setAngle(speakerDistanceAngle.targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case SUBWOOFER_SHOT -> {
        shooter.setState(ShooterState.SUBWOOFER_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.TO_SHOOTER);
      }
      case WAITING_AMP -> {
        shooter.setState(ShooterState.AMP);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);

        snaps.setAngle(SnapManager.getAmpAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case AMP_SHOT -> {
        shooter.setState(ShooterState.AMP);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.TO_SHOOTER);

        snaps.setAngle(SnapManager.getAmpAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
      }
      case UNJAM -> {
        shooter.setState(ShooterState.IDLE);
        intake.setState(IntakeState.OUTTAKE);
        queuer.setState(QueuerState.TO_SHOOTER);
      }
      case WAITING_CLIMB -> {
        shooter.setState(ShooterState.STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
      }
      case CLIMBING -> {
        shooter.setState(ShooterState.STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
      }
      case CLIMBED -> {
        shooter.setState(ShooterState.STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
      }
    }
    climber.setState(newState.climberRaised ? ClimberState.RAISED : ClimberState.LOWERED);
  }

  public void waitAmpRequest() {
    setStateFromRequest(RobotState.WAITING_AMP);
  }

  public void ampRequest() {
    setStateFromRequest(RobotState.AMP_SHOT);
  }

  public void subwooferShotRequest() {
    setStateFromRequest(RobotState.PREPARE_SUBWOOFER_SHOT);
  }

  public void waitSubwooferShotRequest() {
    setStateFromRequest(RobotState.WAITING_SUBWOOFER_SHOT);
  }

  public void floorShotRequest() {
    setStateFromRequest(RobotState.PREPARE_FLOOR_SHOT);
  }

  public void waitFloorShotRequest() {
    setStateFromRequest(RobotState.WAITING_FLOOR_SHOT);
  }

  public void speakerShotRequest() {
    setStateFromRequest(RobotState.SPEAKER_SHOT);
  }

  public void waitSpeakerShotRequest() {
    setStateFromRequest(RobotState.WAITING_SPEAKER_SHOT);
  }

  public void waitSpeakerAutoShotRequest() {
    if (getState().hasNote) {
      setStateFromRequest(RobotState.PREPARE_SPEAKER_SHOT);
    } else {
      intakeRequest();
    }
  }

  public void confirmShotRequest() {
    switch (getState()) {
      case CLIMBED, CLIMBING, WAITING_CLIMB -> {}

      case WAITING_SUBWOOFER_SHOT -> setStateFromRequest(RobotState.PREPARE_SUBWOOFER_SHOT);
      case WAITING_FLOOR_SHOT -> setStateFromRequest(RobotState.PREPARE_FLOOR_SHOT);
      case WAITING_SPEAKER_SHOT -> setStateFromRequest(RobotState.PREPARE_SPEAKER_SHOT);
      case WAITING_SHOOTER_OUTTAKE -> setStateFromRequest(RobotState.PREPARE_SHOOTER_OUTTAKE);
      case WAITING_AMP -> ampRequest();

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

  public void waitOuttakeShooterRequest() {
    setStateFromRequest(RobotState.WAITING_SHOOTER_OUTTAKE);
  }

  public void idleNoGPRequest() {
    setStateFromRequest(RobotState.IDLE_NO_GP);
  }

  public void idleRequest() {
    if (queuer.hasNote()) {
      setStateFromRequest(RobotState.IDLE_W_GP);
    } else {
      setStateFromRequest(RobotState.IDLE_NO_GP);
    }
  }

  public void doClimbSequenceRequest() {
    switch (getState()) {
      default -> setStateFromRequest(RobotState.WAITING_CLIMB);

      case WAITING_CLIMB -> setStateFromRequest(RobotState.CLIMBING);
      case CLIMBING -> {}
      case CLIMBED -> idleRequest();
    }
  }

  public void reverseClimbSequenceRequest() {
    switch (getState()) {
      default -> setStateFromRequest(RobotState.CLIMBED);

      case WAITING_CLIMB -> idleRequest();
      case CLIMBING -> {}
      case CLIMBED -> setStateFromRequest(RobotState.WAITING_CLIMB);
    }
  }

  public void homeClimberRequest() {
    climber.setState(ClimberState.HOMING);
  }

  public void preloadNoteRequest() {
    setStateFromRequest(RobotState.IDLE_W_GP);
  }
}
