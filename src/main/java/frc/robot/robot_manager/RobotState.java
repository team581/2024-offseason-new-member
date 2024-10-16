package frc.robot.robot_manager;

public enum RobotState {
  IDLE_NO_GP,
  INTAKING,
  IDLE_W_GP(true),

  OUTTAKING(true),
  WAITING_SHOOTER_OUTTAKE(true),
  SHOOTER_OUTTAKE(true),

  WAITING_SUBWOOFER_SHOT(true),
  PREPARE_SUBWOOFER_SHOT(true),
  SUBWOOFER_SHOT(true),

  WAITING_FLOOR_SHOT(true),
  PREPARE_FLOOR_SHOT(true),
  FLOOR_SHOT(true),

  WAITING_SPEAKER_SHOT(true),
  PREPARE_SPEAKER_SHOT(true),
  SPEAKER_SHOT(true),

  WAITING_AMP(true),
  AMP_SHOT(true),

  UNJAM,

  WAITING_CLIMB,
  CLIMBING,
  CLIMBED;

  public final boolean hasNote;

  RobotState(boolean hasNote) {
    this.hasNote = hasNote;
  }

  RobotState() {
    this.hasNote = false;
  }
}
