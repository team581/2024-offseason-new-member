package frc.robot.intake_assist;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.vision.LimelightHelpers;

public class IntakeAssistManager {
  private static final double ASSIST_KP = 3.0;
  private static final String LIMELIGHT_NAME = "limelight-note";
  private static final double MAX_ANGLE_CHANGE = -35.0;
  private static final double MIN_ANGLE_CHANGE = 35.0;

  /// tune angle change

  public static ChassisSpeeds getRobotRelativeAssistSpeeds(
      double robotHeading, ChassisSpeeds fieldRelativeInputSpeeds) {

    double tx = LimelightHelpers.getTX(LIMELIGHT_NAME);

    if (tx == 0) {
      return fieldRelativeInputSpeeds;
    }

    DogLog.log("IntakeAssist/TX", tx);

    double fieldRelativeNoteAngle = robotHeading + tx;

    double angleError = robotHeading - fieldRelativeNoteAngle;

    double angleChange = MathUtil.clamp(MIN_ANGLE_CHANGE, MAX_ANGLE_CHANGE, angleError * ASSIST_KP);

    Translation2d requestedFieldRelativeDrive =
        new Translation2d(
            fieldRelativeInputSpeeds.vxMetersPerSecond, fieldRelativeInputSpeeds.vyMetersPerSecond);

    Translation2d newDriveRequest =
        requestedFieldRelativeDrive.rotateBy(Rotation2d.fromDegrees(angleChange));

    return new ChassisSpeeds(
        newDriveRequest.getX(),
        newDriveRequest.getY(),
        fieldRelativeInputSpeeds.omegaRadiansPerSecond);
  }
}
