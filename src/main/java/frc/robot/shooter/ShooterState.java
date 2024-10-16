package frc.robot.shooter;

public enum ShooterState {
  STOPPED(0.0),
  IDLE(100.0),
  SUBWOOFER_SHOT(3000.0),
  FLOOR_SHOT(4800.0),
  SHOOTER_OUTTAKE(500.0),
  SPEAKER_SHOT;

  public final double bottomRPM;
  public final double topRPM;

  ShooterState(double RPM) {
    this.bottomRPM = RPM;
    this.topRPM = RPM;
  }

  ShooterState(double bottomRPM, double topRPM) {
    this.bottomRPM = bottomRPM;
    this.topRPM = topRPM;
  }

  ShooterState() {
    this.bottomRPM = 0.0;
    this.topRPM = 0.0;
  }
}
