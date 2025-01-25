package frc.robot.subsystems.drive;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Pathing {

  public static double aimingFFVelocity(Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
    return pointRelativeSpeeds(currentPose, fieldRelativeSpeeds, 
      targetTranslation
    ).vyMetersPerSecond
      / distanceTo(currentPose, targetTranslation);
  }
  public static double velocityTowards(
    Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
       return pointRelativeSpeeds(currentPose, fieldRelativeSpeeds, 
      targetTranslation
    ).vxMetersPerSecond;
    }
  
  /**
   * Forward is AWAY from the speaker
   * @param currentPose
   * @param fieldRelativeSpeeds
   * @param targetTranslation
   * @return
   */
  public static ChassisSpeeds pointRelativeSpeeds(Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetTranslation) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, 
      headingTo(currentPose, targetTranslation)
    );
  }
  public static Rotation2d headingTo(Pose2d currentPose, Translation2d target) {
    return target.minus(currentPose.getTranslation()).getAngle().minus(new Rotation2d(Math.PI));
  }
  public static double distanceTo(Pose2d currentPose, Translation2d target) {
    return currentPose.getTranslation().minus(target).getNorm();
  }
  public static Pose2d timeAdjustedPose(Pose2d currentPose, ChassisSpeeds robotRelativeSpeeds, double time) {
    return currentPose.transformBy(new Transform2d(
      robotRelativeSpeeds.vxMetersPerSecond * time,
      robotRelativeSpeeds.vyMetersPerSecond*time,
      new Rotation2d(robotRelativeSpeeds.omegaRadiansPerSecond * time)
    ));
  }


}