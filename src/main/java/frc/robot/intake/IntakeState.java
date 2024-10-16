package frc.robot.intake;

public enum IntakeState {
  IDLE(0.0),
  INTAKE(3.0),
  OUTTAKE(-3.0);

  public final double volts;

  IntakeState(double volts) {
    this.volts = volts;
  }
}
