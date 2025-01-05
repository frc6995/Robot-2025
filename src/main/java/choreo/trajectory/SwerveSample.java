// Copyright (c) Choreo contributors

package choreo.trajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.Arrays;

/** A single swerve robot sample in a Trajectory. */
public class SwerveSample {
  private static final double[] EMPTY_MODULE_FORCES = new double[] {0, 0, 0, 0};

  /** The timestamp of this sample, relative to the beginning of the trajectory. */
  public final double t;

  /** The X position of the sample relative to the blue alliance wall origin in meters. */
  public final double x;

  /** The Y position of the sample relative to the blue alliance wall origin in meters. */
  public final double y;

  /** The heading of the sample in radians, with 0 being in the +X direction. */
  public final double heading;

  /** The velocity of the sample in the X direction in m/s. */
  public final double vx;

  /** The velocity of the sample in the Y direction in m/s. */
  public final double vy;

  /** The angular velocity of the sample in rad/s. */
  public final double omega;

  /** The acceleration of the sample in the X direction in m/s². */
  public final double ax;

  /** The acceleration of the sample in the Y direction in m/s². */
  public final double ay;

  /** The angular acceleration of the sample in rad/s². */
  public final double alpha;

  /**
   * The force on each swerve module in the X direction in Newtons. Module forces appear in the
   * following order: [FL, FR, BL, BR].
   */
  private final double[] fx;

  /**
   * The force on each swerve module in the Y direction in Newtons Module forces appear in the
   * following order: [FL, FR, BL, BR].
   */
  private final double[] fy;

  /**
   * Constructs a SwerveSample with the specified parameters.
   *
   * @param t The timestamp of this sample, relative to the beginning of the trajectory.
   * @param x The X position of the sample in meters.
   * @param y The Y position of the sample in meters.
   * @param heading The heading of the sample in radians, with 0 being in the +X direction.
   * @param vx The velocity of the sample in the X direction in m/s.
   * @param vy The velocity of the sample in the Y direction in m/s.
   * @param omega The angular velocity of the sample in rad/s.
   * @param ax The acceleration of the sample in the X direction in m/s².
   * @param ay The acceleration of the sample in the Y direction in m/s².
   * @param alpha The angular acceleration of the sample in rad/s².
   * @param moduleForcesX The force on each swerve module in the X direction in Newtons. Module
   *     forces appear in the following order: [FL, FR, BL, BR].
   * @param moduleForcesY The force on each swerve module in the Y direction in Newtons. Module
   *     forces appear in the following order: [FL, FR, BL, BR].
   */
  public SwerveSample(
      double t,
      double x,
      double y,
      double heading,
      double vx,
      double vy,
      double omega,
      double ax,
      double ay,
      double alpha,
      double[] moduleForcesX,
      double[] moduleForcesY) {
    this.t = t;
    this.x = x;
    this.y = y;
    this.heading = heading;
    this.vx = vx;
    this.vy = vy;
    this.omega = omega;
    this.ax = ax;
    this.ay = ay;
    this.alpha = alpha;
    this.fx = moduleForcesX;
    this.fy = moduleForcesY;
  }

  /**
   * A null safe getter for the module forces in the X direction.
   *
   * @return The module forces in the X direction.
   */
  public double[] moduleForcesX() {
    if (fx == null || fx.length != 4) {
      return EMPTY_MODULE_FORCES;
    }
    return fx;
  }

  /**
   * A null safe getter for the module forces in the Y direction.
   *
   * @return The module forces in the Y direction.
   */
  public double[] moduleForcesY() {
    if (fy == null || fy.length != 4) {
      return EMPTY_MODULE_FORCES;
    }
    return fy;
  }

  public double getTimestamp() {
    return t;
  }

  public Pose2d getPose() {
    return new Pose2d(x, y, Rotation2d.fromRadians(heading));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(vx, vy, omega);
  }

  /** The struct for the SwerveSample class. */
  public static final Struct<SwerveSample> struct = new SwerveSampleStruct();

  private static final class SwerveSampleStruct implements Struct<SwerveSample> {
    @Override
    public Class<SwerveSample> getTypeClass() {
      return SwerveSample.class;
    }

    @Override
    public String getTypeName() {
      return "SwerveSample";
    }

    @Override
    public int getSize() {
      return Struct.kSizeDouble * 18;
    }

    @Override
    public String getSchema() {
      return "double timestamp;"
          + "Pose2d pose;"
          + "double vx;"
          + "double vy;"
          + "double omega;"
          + "double ax;"
          + "double ay;"
          + "double alpha;"
          + "double moduleForcesX[4];"
          + "double moduleForcesY[4];";
    }

    @Override
    public Struct<?>[] getNested() {
      return new Struct<?>[] {Pose2d.struct};
    }

    @Override
    public SwerveSample unpack(ByteBuffer bb) {
      return new SwerveSample(
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble(),
          bb.getDouble(),
          new double[] {bb.getDouble(), bb.getDouble(), bb.getDouble(), bb.getDouble()},
          new double[] {bb.getDouble(), bb.getDouble(), bb.getDouble(), bb.getDouble()});
    }

    @Override
    public void pack(ByteBuffer bb, SwerveSample value) {
      bb.putDouble(value.t);
      bb.putDouble(value.x);
      bb.putDouble(value.y);
      bb.putDouble(value.heading);
      bb.putDouble(value.vx);
      bb.putDouble(value.vy);
      bb.putDouble(value.omega);
      bb.putDouble(value.ax);
      bb.putDouble(value.ay);
      bb.putDouble(value.alpha);
      for (int i = 0; i < 4; ++i) {
        bb.putDouble(value.moduleForcesX()[i]);
      }
      for (int i = 0; i < 4; ++i) {
        bb.putDouble(value.moduleForcesY()[i]);
      }
    }
  }

  @Override
  public boolean equals(Object obj) {
    if (!(obj instanceof SwerveSample)) {
      return false;
    }

    var other = (SwerveSample) obj;
    return this.t == other.t
        && this.x == other.x
        && this.y == other.y
        && this.heading == other.heading
        && this.vx == other.vx
        && this.vy == other.vy
        && this.omega == other.omega
        && this.ax == other.ax
        && this.ay == other.ay
        && this.alpha == other.alpha;
  }
}
