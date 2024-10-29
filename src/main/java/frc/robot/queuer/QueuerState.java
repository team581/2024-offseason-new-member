package frc.robot.queuer;

public enum QueuerState {
  IDLE(0.0),
  TO_SHOOTER(5.0),
  TO_INTAKE(-2.0);

  public final double volts;

  QueuerState(double volts) {
    this.volts = volts;
  }
}
