package frc.robot.shooter;

public enum ShooterState {
  STOPPED(0.0),
  IDLE(200.0),
  SUBWOOFER_SHOT(3000.0),
  MANUAL_SHOT(2500.0),
  FLOOR_SHOT(4000.0),
  SHOOTER_OUTTAKE(700.0),
  AMP(1600),
  SPEAKER_SHOT;

  public final double bottomRPM;
  public final double topRPM;

  ShooterState(double rpm) {
    this(rpm, rpm);
  }

  ShooterState(double topRPM, double bottomRPM) {
    this.bottomRPM = bottomRPM;
    this.topRPM = topRPM;
  }

  ShooterState() {
    this(0);
  }
}
