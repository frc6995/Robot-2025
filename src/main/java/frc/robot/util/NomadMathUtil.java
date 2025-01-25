package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

public class NomadMathUtil {

  public static double FIELD_WIDTH = 8.0137;
  public static double FIELD_LENGTH = 16.54175;

  public static Rotation2d getDirection(Transform2d transform) {
    return getDirection(transform.getTranslation());
  }

  public static Rotation2d getDirection(Translation2d transform) {
    // add tiny number so that 0/0 comes out to 0 angle, not a div by 0 error
    return new Rotation2d(transform.getX(), transform.getY());
  }

  public static Rotation2d getDirection(Pose2d tail, Pose2d head) {
    return getDirection(head.getTranslation().minus(tail.getTranslation()));
  }

  public static double getDistance(Transform2d transform) {
    return getDistance(transform.getTranslation());
  }

  public static double getDistance(Translation2d transform) {
    return transform.getNorm();
  }

  public static double subtractkS(double voltage, double kS) {
    if (Math.abs(voltage) <= kS) {
      voltage = 0;
    } else {
      voltage -= Math.copySign(kS, voltage);
    }
    return voltage;
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle, double flipThreshold) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > flipThreshold) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }

  public static Pose2d mirrorPose(Pose2d bluePose) {
    return new Pose2d(
        FIELD_LENGTH - bluePose.getX(),
        bluePose.getY(),
        Rotation2d.fromRadians(Math.PI - bluePose.getRotation().getRadians()));
  }

  public static Pose2d mirrorPose(Pose2d bluePose, DriverStation.Alliance alliance) {
    if (alliance != Alliance.Red) {
      return bluePose;
    }
    return mirrorPose(bluePose);
  }

  public static Translation2d mirrorTranslation(Translation2d blueTranslation) {
    return new Translation2d(
        FIELD_LENGTH - blueTranslation.getX(),
        blueTranslation.getY());
  }

  public static Translation2d mirrorTranslation(Translation2d blueTranslation, DriverStation.Alliance alliance) {
    if (alliance != Alliance.Red) {
      return blueTranslation;
    }
    return mirrorTranslation(blueTranslation);
  }

  
}
