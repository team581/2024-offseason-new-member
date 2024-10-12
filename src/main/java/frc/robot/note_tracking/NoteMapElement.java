package frc.robot.note_tracking;

import edu.wpi.first.math.geometry.Translation2d;

public record NoteMapElement(double expiresAt, Translation2d noteTranslation, int health) {
  public NoteMapElement(double expiry, Translation2d translation) {
    this(expiry, translation, 10);
  }
}
