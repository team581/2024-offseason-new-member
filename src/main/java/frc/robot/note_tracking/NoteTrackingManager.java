package frc.robot.note_tracking;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto_manager.AutoNoteDropped;
import frc.robot.auto_manager.BoundingBox;
import frc.robot.config.RobotConfig;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.snaps.SnapManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.DistanceAngle;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class NoteTrackingManager extends LifecycleSubsystem {
  private static final double CAMERA_IMAGE_HEIGHT = 960.0;
  private static final double CAMERA_IMAGE_WIDTH = 1280.0;
  // how much we keep a note in the map if it was added or updated from camera (seconds)
  private static final double NOTE_MAP_LIFETIME = 10.0;
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final RobotCommands actions;
  private final RobotManager robot;
  private final SnapManager snaps;
  private static final String LIMELIGHT_NAME = "limelight-note";
  private static final NetworkTableEntry LL_TCORNXY =
      NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("tcornxy");
  private final InterpolatingDoubleTreeMap tyToDistance = new InterpolatingDoubleTreeMap();
  private ArrayList<NoteMapElement> noteMap = new ArrayList<>();

  private static final double FOV_VERTICAL = 48.823;
  private static final double FOV_HORIZONTAL = 62.074;
  private static final double HORIZONTAL_LEFT_VIEW = 27.491;
  private static final double VERTICAL_TOP_VIEW = 24.955;

  private static final BoundingBox ROBOT_RELATIVE_FOV_BOUNDS =
      new BoundingBox(
          // top left
          new Translation2d(-1.250, -0.5),
          // top right
          new Translation2d(-1.250, 0.5),
          // bottom left
          new Translation2d(-0.725, -0.1),
          // bottom right
          new Translation2d(-0.720, 0.1));

  public NoteTrackingManager(
      LocalizationSubsystem localization,
      SwerveSubsystem swerve,
      RobotCommands actions,
      RobotManager robot) {
    super(SubsystemPriority.NOTE_TRACKING);

    this.localization = localization;
    this.swerve = swerve;
    this.actions = actions;
    this.robot = robot;
    this.snaps = robot.snaps;
    RobotConfig.get().vision().tyToNoteDistance().accept(tyToDistance);
  }

  private boolean noteInView(Translation2d fieldRelativeNote) {
    var robotRelativeNote = getRobotRelativeNote(fieldRelativeNote);
    DogLog.log(
        "NoteTracking/Debug/BoxContainsNote",
        ROBOT_RELATIVE_FOV_BOUNDS.contains(robotRelativeNote));
    return ROBOT_RELATIVE_FOV_BOUNDS.contains(robotRelativeNote);
  }

  private Translation2d getRobotRelativeNote(Translation2d fieldRelativeNote) {
    var robotPose = localization.getPose();
    Rotation2d negativeRobotRotation = robotPose.getRotation().unaryMinus();
    var robotRelativeNoteTranslation =
        fieldRelativeNote.minus(robotPose.getTranslation()).rotateBy(negativeRobotRotation);
    return robotRelativeNoteTranslation;
  }

  public void resetNoteMap(ArrayList<NoteMapElement> startingValues) {
    AutoNoteDropped.clearDroppedNotes();
    noteMap = startingValues;
  }

  /**
   * Get the pose of the note closest to the provided location, within a threshold. Returns
   * optional.empty if no notes are tracked or notes exceed the threshold.
   */
  public Optional<NoteMapElement> getNoteNearPose(
      Translation2d searchLocation, double thresholdMeters) {

    var maybeElement =
        noteMap.stream()
            .filter(
                element -> {
                  return element.noteTranslation().getDistance(searchLocation) < thresholdMeters;
                })
            .min(
                (a, b) ->
                    Double.compare(
                        a.noteTranslation().getDistance(searchLocation),
                        b.noteTranslation().getDistance(searchLocation)));

    if (!maybeElement.isPresent()) {
      return Optional.empty();
    }

    return Optional.of(maybeElement.get());
  }

  private Optional<Pose2d> noteTxTyToPose(double tx, double ty) {

    double latency =
        (LimelightHelpers.getLatency_Capture(LIMELIGHT_NAME)
                + LimelightHelpers.getLatency_Pipeline(LIMELIGHT_NAME))
            / 1000.0;
    double timestamp = Timer.getFPGATimestamp() - latency;

    Pose2d robotPoseAtCapture = localization.getPose(timestamp);

    if (tx == 0) {
      return Optional.empty();
    }

    double forwardDistanceToNote = tyToDistance.get(ty);
    Rotation2d angleFromNote = Rotation2d.fromDegrees(tx);

    var c = forwardDistanceToNote / Math.cos(angleFromNote.getRadians());
    double sidewaysDistanceToNote = Math.sqrt(Math.pow(c, 2) - Math.pow(forwardDistanceToNote, 2));

    // Flips side of robot note is on based on if tx is positive or negative
    if (tx > 0) {
      sidewaysDistanceToNote *= -1.0;
    }

    var notePoseWithoutRotation =
        new Translation2d(-forwardDistanceToNote, -sidewaysDistanceToNote)
            .rotateBy(robotPoseAtCapture.getRotation());

    var notePoseWithRobot =
        new Translation2d(
            robotPoseAtCapture.getX() + notePoseWithoutRotation.getX(),
            robotPoseAtCapture.getY() + notePoseWithoutRotation.getY());
    // Uses distance angle math to aim, inverses the angle for intake

    DistanceAngle noteDistanceAngle =
        VisionSubsystem.distanceAngleToTarget(
            new Pose2d(notePoseWithRobot, new Rotation2d()), robotPoseAtCapture);
    Rotation2d rotation =
        new Rotation2d(Units.degreesToRadians(noteDistanceAngle.targetAngle()) + Math.PI);

    return Optional.of(new Pose2d(notePoseWithRobot, rotation));
  }

  private List<Pose2d> getRawNotePoses() {
    List<Pose2d> notePoses = new ArrayList<>();
    double[] corners = LL_TCORNXY.getDoubleArray(new double[0]);
    // Loop through 4 points

    // Delete 3 point note data

    if (corners.length >= 8 && corners[0] != 0.0 && corners.length % 8 == 0) {

      for (int i = 0; i < corners.length; i = i + 8) {
        var centerX = (corners[i] + corners[i + 2]) / 2.0;
        var centerY = (corners[i + 1] + corners[i + 5]) / 2.0;

        double angleX = (((centerX / CAMERA_IMAGE_WIDTH) * FOV_HORIZONTAL) - HORIZONTAL_LEFT_VIEW);
        double angleY =
            -1.0 * (((centerY / CAMERA_IMAGE_HEIGHT) * FOV_VERTICAL) - VERTICAL_TOP_VIEW);

        var maybeNotePose = noteTxTyToPose(angleX, angleY);

        if (maybeNotePose.isPresent()) {
          notePoses.add(maybeNotePose.get());
        }
      }
    }
    return notePoses;
  }

  private static boolean isOutOfBounds(Pose2d notePose) {
    var fieldBorderThreshold = Units.inchesToMeters(4);
    boolean yOutOfBounds =
        notePose.getY() < (0.0 + fieldBorderThreshold)
            || notePose.getY() > (8.2 - fieldBorderThreshold);
    boolean xOutofBounds =
        notePose.getX() < (0.0 + fieldBorderThreshold)
            || notePose.getX() > (16.51 - fieldBorderThreshold);

    return yOutOfBounds || xOutofBounds;
  }

  private List<Pose2d> getFilteredNotePoses() {
    if (!safeToTrack()) {
      return List.of();
    }
    List<Pose2d> possibleNotes = getRawNotePoses();
    List<Pose2d> filteredNotes = new ArrayList<>();

    for (Pose2d possibleNote : possibleNotes) {
      if (!isOutOfBounds(possibleNote)) {
        filteredNotes.add(possibleNote);
      }
    }

    return filteredNotes;
  }

  private boolean safeToTrack() {
    var speeds = swerve.getRobotRelativeSpeeds();

    return speeds.vxMetersPerSecond < 3
        && speeds.vyMetersPerSecond < 3
        && speeds.omegaRadiansPerSecond < Units.degreesToRadians(3.0);
  }

  private void removeNote(Translation2d searchArea, double threshold) {
    var intakedNote = getNoteNearPose(searchArea, 1.5);
    if (intakedNote.isPresent()) {
      noteMap.remove(intakedNote.get());
    }
  }

  public Command intakeNoteAtPose(Translation2d searchPose, double thresholdMeters) {
    return intakeNoteAtPose(() -> Optional.of(searchPose), thresholdMeters);
  }

  public Command intakeNoteAtPose(
      Supplier<Optional<Translation2d>> maybeSearchPoseSupplier, double thresholdMeters) {
    return actions
        .intakeCommand()
        .alongWith(
            swerve.driveToPoseCommand(
                () -> {
                  var maybeSearchPose = maybeSearchPoseSupplier.get();
                  if (maybeSearchPose.isEmpty()) {
                    return Optional.empty();
                  }

                  var nearestNote = getNoteNearPose(maybeSearchPose.get(), thresholdMeters);

                  if (nearestNote.isPresent()) {
                    DistanceAngle noteDistanceAngle =
                        VisionSubsystem.distanceAngleToTarget(
                            new Pose2d(nearestNote.get().noteTranslation(), new Rotation2d()),
                            localization.getPose());
                    Rotation2d rotation =
                        new Rotation2d(
                            Units.degreesToRadians(noteDistanceAngle.targetAngle()) + Math.PI);
                    var notePose = new Pose2d(nearestNote.get().noteTranslation(), rotation);

                    snaps.setAngle(rotation.getDegrees());
                    snaps.setEnabled(true);

                    return Optional.of(notePose);

                  } else {
                    snaps.setEnabled(false);
                    return Optional.empty();
                  }
                },
                localization::getPose,
                false))
        .until(
            () -> {
              var maybeSearchPose = maybeSearchPoseSupplier.get();
              if (maybeSearchPose.isEmpty()) {
                // Exit because the search pose is not there
                return true;
              }
              var searchPose = maybeSearchPose.get();
              var intakedNote = getNoteNearPose(searchPose, 1.5);
              if (robot.getState().hasNote) {
                // Already holding note
                // We just intaked this note, so let's remove it from note map
                if (intakedNote.isPresent()) {
                  removeNote(intakedNote.get().noteTranslation(), 1.5);
                }
                return true;
              }

              if (getNoteNearPose(searchPose, thresholdMeters).isEmpty()) {
                // No note tracked within the search area, it must be gone, so continue
                return true;
              }

              if (localization.atTranslation(searchPose, 0.05)) {
                // We have driven to where the note is, but don't have it
                // We should have intaked the note at this point, so just continue

                // Someone else probably just intaked this note, so let's remove it from note map
                DogLog.log("NoteTracking/TimeoutAtTranslation", Timer.getFPGATimestamp());
                if (intakedNote.isPresent()) {
                  removeNote(intakedNote.get().noteTranslation(), 1.5);
                }
                return true;
              }
              return false;
            })
        // .andThen(
        //     Commands.runOnce(
        //         () -> {
        //           var maybeSearchPose = maybeSearchPoseSupplier.get();
        //           if (maybeSearchPose.isEmpty()) {
        //             return;
        //           }
        //           var intakedNote = getNoteNearPose(maybeSearchPose.get(), 1.5);
        //           if (intakedNote.isPresent()) {
        //             removeNote(intakedNote.get());
        //           }
        //         }))
        .finallyDo(
            () -> {
              snaps.setEnabled(false);
            })
        .withTimeout(4.0)
        .withName("IntakeNearestNoteCommand");
  }

  public Command intakeNearestMapNote(double thresholdMeters) {
    return intakeNoteAtPose(
        () -> Optional.of(localization.getPose().getTranslation()), thresholdMeters);
  }

  @Override
  public void robotPeriodic() {

    if (DriverStation.isTeleop() && !RobotConfig.get().perfToggles().noteMapInTeleop()) {
      return;
    }

    updateMap();

    try {
      DogLog.log(
          "NoteTracking/NoteMap",
          noteMap.stream()
              .map(element -> new Pose2d(element.noteTranslation(), new Rotation2d()))
              .toArray(Pose2d[]::new));
    } catch (Exception error) {
      DogLog.logFault("NoteMapLoggingError");
      System.err.println(error);
    }

    var fieldRelativeBounds = getFieldRelativeBounds();
    DogLog.log("NoteTracking/CameraBounds", fieldRelativeBounds.toArray(Pose2d[]::new));
  }

  private List<Pose2d> getFieldRelativeBounds() {
    var robotRelativeToFieldRelativeTransform =
        new Transform2d(new Pose2d(), localization.getPose());
    return ROBOT_RELATIVE_FOV_BOUNDS.getPoints().stream()
        .map(point -> point.plus(robotRelativeToFieldRelativeTransform))
        .toList();
  }

  public boolean mapContainsNote() {
    return !noteMap.isEmpty();
  }

  public void addNoteToMap(Translation2d pose) {
    noteMap.add(new NoteMapElement(Timer.getFPGATimestamp() + NOTE_MAP_LIFETIME, pose, 10));
  }

  private void updateMap() {
    List<Pose2d> visionNotes = getFilteredNotePoses();

    noteMap.removeIf(
        element -> {
          return (element.expiresAt() < Timer.getFPGATimestamp());
        });

    if (RobotConfig.get().perfToggles().noteMapBoundingBox() && safeToTrack()) {

      var filteredNotesInBox =
          noteMap.stream()
              .filter(
                  element -> {
                    return (noteInView(element.noteTranslation()));
                  })
              .toList();

      for (NoteMapElement noteMapElement : filteredNotesInBox) {
        noteMap.remove(noteMapElement);
        if (noteMapElement.health() > 1) {
          noteMap.add(
              new NoteMapElement(
                  noteMapElement.expiresAt(),
                  noteMapElement.noteTranslation(),
                  noteMapElement.health() - 1));
        }
      }
    }

    double newNoteExpiry = Timer.getFPGATimestamp() + NOTE_MAP_LIFETIME;

    for (var visionNote : visionNotes) {
      Optional<NoteMapElement> match =
          noteMap.stream()
              .filter(
                  rememberedNote -> {
                    return rememberedNote.expiresAt() != newNoteExpiry
                        && (rememberedNote
                                .noteTranslation()
                                .getDistance(visionNote.getTranslation())
                            < 1.0);
                  })
              .min(
                  (a, b) ->
                      Double.compare(
                          a.noteTranslation().getDistance(visionNote.getTranslation()),
                          b.noteTranslation().getDistance(visionNote.getTranslation())));

      if (match.isPresent()) {
        noteMap.remove(match.get());
      }

      noteMap.add(new NoteMapElement(newNoteExpiry, visionNote.getTranslation()));
    }
  }
}
