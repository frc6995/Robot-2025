package frc.robot.util.setpointGenerator3173;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import wpilibExt.Speeds.RobotSpeeds;


public record SwerveSetpoint(
    /**Robot-relative */
    RobotSpeeds speeds, AdvancedSwerveModuleState[] moduleStates) {
  public static SwerveSetpoint zeroed() {
    return new SwerveSetpoint(
        RobotSpeeds.kZero,
        new AdvancedSwerveModuleState[] {
          new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0),
          new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0),
          new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0),
          new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0)
        });
  }
}
