package frc.robot.intake;

public enum IntakeState {
  IDLE(0.0),
  INTAKE(4.0),
  EXPECT_NOTE(3.0),
  TO_SHOOTER(10.0),
  OUTTAKE(-6.0);

  public final double volts;

  IntakeState(double volts) {
    this.volts = volts;
  }
}
