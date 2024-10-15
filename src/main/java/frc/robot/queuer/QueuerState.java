package frc.robot.queuer;

public enum QueuerState {
  IDLE(0.0),
  TO_SHOOTER(12.0),
  TO_INTAKE(-1.0);

  public final double volts;
  QueuerState(double volts) {
    this.volts = volts;
  }
}
