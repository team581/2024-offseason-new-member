package frc.robot.queuer;

public enum QueuerState {
  IDLE(0.0),
  INTAKING(3.0),
  TO_SHOOTER(10.0),
  TO_INTAKE(-6.0);

  public final double volts;

  QueuerState(double volts) {
    this.volts = volts;
  }
}
