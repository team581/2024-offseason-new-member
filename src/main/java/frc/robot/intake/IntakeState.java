package frc.robot.intake;

public enum IntakeState {
  IDLE(0.0, false),
  INTAKE(3.0, false),
  OUTTAKE(-3.0, false);

  public final double volts;
  public final boolean inverted;

  IntakeState(double volts, boolean funnelInvert) {
    this.volts = volts;
    this.inverted = funnelInvert;
  }
}
