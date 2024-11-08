package frc.robot.autos;

public enum AutoSelection {
  DO_NOTHING("", ""),
  FOUR_PIECE_SUBWOOFER("Red 4 Piece", "Blue 4 Piece"),
  SHOOT_STOP_AMP("Red Shoot Stop Amp", "Blue Shoot Stop Amp"),
  SHOOT_STOP_SOURCE("Red Shoot Stop Source", "Blue Shoot Stop Source"),
  THREE_PIECE_MID_SOURCE("Red 3 Piece Source", "Blue 3 Piece Source");

  public final String redAutoName;
  public final String blueAutoName;

  private AutoSelection(String redAutoName, String blueAutoName) {
    this.redAutoName = redAutoName;
    this.blueAutoName = blueAutoName;
  }
}
