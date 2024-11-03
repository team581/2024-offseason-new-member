package frc.robot.swerve;

import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import java.util.List;

public class SnapUtil {

  public static double getAmpAngle() {
    return FmsSubsystem.isRedAlliance() ? 90 : (-90.0);
  }

  public static double getPodiumAngle() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 0 : (180.0);
  }

  public static double getSubwooferAngle() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 0 : (180.0);
  }

  public static double getSourceAngle() {
    return FmsSubsystem.isRedAlliance() ? 60.0 : (180.0 - 60.0);
  }

  public static double getStageLeftAngle() {
    return FmsSubsystem.isRedAlliance() ? -60 : (180.0 - 60);
  }

  public static double getStageRightAngle() {
    return FmsSubsystem.isRedAlliance() ? 60 : (180.0 + 60);
  }

  public static double getStageBackChainAngle() {
    return FmsSubsystem.isRedAlliance() ? 180 : (0);
  }

  private static final List<Double> RED_STAGE_ANGLES = List.of(-60.0, 60.0, 180.0);
  private static final List<Double> BLUE_STAGE_ANGLES = List.of(-60.0 + 180.0, 60.0 + 180.0, 0.0);

  public static double getClimbingAngle(ImuSubsystem imu) {
    var usedAngles = FmsSubsystem.isRedAlliance() ? RED_STAGE_ANGLES : BLUE_STAGE_ANGLES;

    var currentAngle = imu.getRobotHeading();

    return usedAngles.stream()
        .min((a, b) -> Double.compare(currentAngle - a, currentAngle - b))
        .orElse(0.0);
  }

  private SnapUtil() {}
}
