package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;

public class AutoCommands {
  private static final boolean USE_DYNAMIC_AUTOS = true;

  private static Command followPathForAlliance(PathPlannerPath redPath, PathPlannerPath bluePath) {
    return Commands.either(
        AutoBuilder.followPath(redPath),
        AutoBuilder.followPath(bluePath),
        FmsSubsystem::isRedAlliance);
  }

  private final RobotCommands actions;
  private final RobotManager robotManager;

  public AutoCommands(RobotCommands actions, RobotManager robotManager) {
    this.actions = actions;
    this.robotManager = robotManager;
  }

  public Command doNothingCommand() {
    return robotManager.localization.getZeroCommand();
  }

  public Command speakerSnapCommand() {
    return actions.waitSpeakerCommand();
  }

  public Command subwooferShotWithTimeout() {
    return actions
        .subwooferShotCommand()
        .withTimeout(2)
        .andThen(actions.subwooferShotCommand().withTimeout(1))
        .withName("SubwooferShotWithTimeout");
  }

  public Command speakerShotWithTimeout() {
    return actions
        .speakerShotCommand()
        .withTimeout(2)
        .andThen(actions.speakerShotCommand().withTimeout(1))
        .withName("SpeakerShotWithTimeout");
  }

  private boolean hasNote() {
    if (!USE_DYNAMIC_AUTOS) {
      return true;
    }

    return robotManager.queuer.hasNote() || robotManager.getState().hasNote;
  }

  // public Command doManyAutoSteps(List<AutoNoteStep> steps) {
  //   return
  // Commands.sequence(steps.stream().map(autoManager::doAutoStep).toArray(Command[]::new));
  // }

  // public Command noteMap456Command() {
  //   return doManyAutoSteps(
  //       List.of(AutoNoteStep.score(4, 5), AutoNoteStep.score(5, 6), AutoNoteStep.score(6)));
  // }

  // public Command notemap567Command() {
  //   return Commands.sequence(
  //       Commands.runOnce(
  //           () -> {
  //             var now = Timer.getFPGATimestamp();
  //             noteTrackingManager.resetNoteMap(
  //                 new ArrayList<>(
  //                     List.of(
  //                         new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(5)),
  //                         new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(6)),
  //                         new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(7)))));
  //           }),
  //       doManyAutoSteps(
  //           List.of(AutoNoteStep.score(5, 6), AutoNoteStep.score(6, 7), AutoNoteStep.score(7))));
  // }

  // public Command notemap4_10Command() {
  //   return doManyAutoSteps(
  //       List.of(AutoNoteStep.score(4, 5), AutoNoteStep.score(5, 6), AutoNoteStep.score(10)));
  // }

  // public Command noteMapResetCommand() {
  //   return Commands.runOnce(
  //       () -> {
  //         var now = Timer.getFPGATimestamp();
  //         noteTrackingManager.resetNoteMap(
  //             new ArrayList<>(
  //                 List.of(
  //                     new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(4)),
  //                     new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(1)),
  //                     new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(2)),
  //                     new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(3)),
  //                     new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(5)),
  //                     new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(6)),
  //                     new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(7)),
  //                     new NoteMapElement(now + 10, AutoNoteStaged.noteIdToTranslation(8)))));
  //       });
  // }

  public Command waitingDropRequestCommand() {
    return Commands.runOnce(
        () -> {
          robotManager.waitOuttakeShooterRequest();
        });
  }
}
