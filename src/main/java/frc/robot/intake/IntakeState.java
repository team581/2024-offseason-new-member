package frc.robot.intake;

public enum IntakeState {
  IDLE(0.0),
  INTAKE(4.0),
  OUTTAKE(-2.0);

  public final double volts;

  IntakeState(double volts) {
    this.volts = volts;
  }
}
