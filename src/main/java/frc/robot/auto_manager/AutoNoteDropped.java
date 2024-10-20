package frc.robot.auto_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class AutoNoteDropped {
  private static int nextDroppedNoteId = 0;

  public static void clearDroppedNotes() {
    droppedNoteIdToPose.clear();
    nextDroppedNoteId = 0;
    DogLog.log("Debug/ClearedNoteNextId", nextDroppedNoteId);
    DogLog.log("Debug/ClearedNoteHashMap", droppedNoteIdToPose.toString());
    // log next dropped note id
    // log translation
    // log size of the map
  }

  public static void addDroppedNote(Translation2d translation) {

    DogLog.log("Debug/AddDroppedNoteBeforeAdding", nextDroppedNoteId);
    DogLog.log("Debug/HashMapBeforeAdding", droppedNoteIdToPose.toString());
    droppedNoteIdToPose.put(nextDroppedNoteId++, translation);
    DogLog.log("Debug/AddDroppedNoteAfterAdding", nextDroppedNoteId);
    DogLog.log("Debug/HashMapAfterAdding", droppedNoteIdToPose.toString());
    // log next dropped note id
    // log translation
    // log size of the map
  }

  private static Map<Integer, Translation2d> droppedNoteIdToPose = new HashMap<>();

  private final int droppedNoteId;

  public AutoNoteDropped(int droppedNoteId) {
    this.droppedNoteId = droppedNoteId;
  }

  public Optional<Translation2d> getPose() {
    // log stuff here also if its useful

    DogLog.log("Debug/ContainsKey", droppedNoteIdToPose.containsKey(droppedNoteId));
    DogLog.log("Debug/ContainsKeyId", droppedNoteId);
    var value = droppedNoteIdToPose.get(droppedNoteId);
    if (value == null) {
      DogLog.log("Debug/Value", "null :(");

    } else {
      DogLog.log("Debug/Value", value);
    }

    if (droppedNoteIdToPose.containsKey(droppedNoteId)) {
      return Optional.of(droppedNoteIdToPose.get(droppedNoteId));
    }

    return Optional.empty();
  }
}
