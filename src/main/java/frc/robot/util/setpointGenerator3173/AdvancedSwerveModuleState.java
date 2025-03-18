package frc.robot.util.setpointGenerator3173;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;

public class AdvancedSwerveModuleState extends SwerveModuleState {
  public double steerVelocity;
  public double driveAcceleration;
  public double wheelForceX;
  public double wheelForceY;
  public AdvancedSwerveModuleState(
      double speedMetersPerSecond, Rotation2d angle, double driveMetersPerSec2) {
    super(speedMetersPerSecond, angle);
    this.driveAcceleration = driveMetersPerSec2;
    double wheelAccelerationX = driveAcceleration * angle.getCos();
    double wheelAccelerationY = driveAcceleration * angle.getCos();
  }

  // todo: implement custom struct
  public static final Struct<SwerveModuleState> struct = SwerveModuleState.struct;

  public static AdvancedSwerveModuleState fromBase(SwerveModuleState base) {
    return new AdvancedSwerveModuleState(base.speedMetersPerSecond, base.angle, 0.0);
  }
}
