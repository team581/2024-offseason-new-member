package frc.robot.swerve;

import frc.robot.fms.FmsSubsystem;

public class SnapUtil {
  public static double getAmpAngle() {
    return FmsSubsystem.isRedAlliance() ? 90 : 90.0 - 180.0;
  }

  public static double getSubwooferAngle() {
    return FmsSubsystem.isRedAlliance() ? 0 : 180.0;
  }

  private SnapUtil() {}
}
