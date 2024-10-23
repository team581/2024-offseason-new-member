package frc.robot.autos;

public enum AutoSelection {
  DO_NOTHING("", ""),
  FOUR_PIECE_SUBWOOFER("4_piece_red_sub","4_piece_blue_sub"),
  SHOOT_STOP_AMP("shoot_stop_red_amp","shoot_stop_blue_amp"),
  SHOOT_STOP_SOURCE("shoot_stop_red_source","shoot_stop_blue_source");

  public final String redAutoName;
  public final String blueAutoName;

  private AutoSelection(String redAutoName, String blueAutoName) {
    this.redAutoName = redAutoName;
    this.blueAutoName = blueAutoName;
  }
}
