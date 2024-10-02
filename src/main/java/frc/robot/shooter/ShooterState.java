package frc.robot.shooter;

public enum ShooterState {
    STOPPED(0.0),
    IDLE(100.0),
    SUBWOOFER_SHOT(3000.0),
    FLOOR_SHOT(4800.0);

    public final double RPM;
    ShooterState(double RPM) {
        this.RPM = RPM;
    }
}
