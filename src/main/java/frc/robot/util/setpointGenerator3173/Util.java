package frc.robot.util.setpointGenerator3173;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.util.setpointGenerator3173.Types.ChassisConstraints;

class Util {
  private static final double kEpsilon = 1E-5;
  static final int NUM_MODULES = 4;

  static double unwrapAngle(double ref, double angle) {
    double diff = angle - ref;
    if (diff > Math.PI) {
      return angle - 2.0 * Math.PI;
    } else if (diff < -Math.PI) {
      return angle + 2.0 * Math.PI;
    } else {
      return angle;
    }
  }

  @FunctionalInterface
  interface Function2d {
    double f(double x, double y);
  }

  static double findSteeringMaxS(
      double prevVX,
      double prevVY,
      double prevTheta,
      double desiredVX,
      double desiredVY,
      double desiredTheta,
      double maxThetaStepSize) {
    desiredTheta = unwrapAngle(prevTheta, desiredTheta);
    double diff = desiredTheta - prevTheta;
    if (Math.abs(diff) <= maxThetaStepSize) {
      // Can go all the way to s=1.
      return 1.0;
    }

    double x_0 = prevVX;
    double y_0 = prevVY;
    double theta_0 = prevTheta;
    double x_1 = desiredVX;
    double y_1 = desiredVY;

    double target = theta_0 + Math.copySign(maxThetaStepSize, diff);

    // Rotate the velocity vectors such that the target angle becomes the +X
    // axis. We only need find the Y components, h_0 and h_1, since they are
    // proportional to the distances from the two points to the solution
    // point (x_0 + (x_1 - x_0)s, y_0 + (y_1 - y_0)s).
    double sin = Math.sin(-target);
    double cos = Math.cos(-target);
    double h_0 = sin * x_0 + cos * y_0;
    double h_1 = sin * x_1 + cos * y_1;

    // Undo linear interpolation from h_0 to h_1:
    // 0 = h_0 + (h_1 - h_0) * s
    // -h_0 = (h_1 - h_0) * s
    // -h_0 / (h_1 - h_0) = s
    // h_0 / (h_0 - h_1) = s
    // Guaranteed to not divide by zero, since if h_0 was equal to h_1, theta_0
    // would be equal to theta_1, which is caught by the difference check.
    return h_0 / (h_0 - h_1);
  }

  private static boolean isValidS(double s) {
    return Double.isFinite(s) && s >= 0 && s <= 1;
  }

  static double findDriveMaxS(double x_0, double y_0, double x_1, double y_1, double maxVelStep) {
    // Derivation:
    // Want to find point P(s) between (x_0, y_0) and (x_1, y_1) where the
    // length of P(s) is the target T. P(s) is linearly interpolated between the
    // points, so P(s) = (x_0 + (x_1 - x_0) * s, y_0 + (y_1 - y_0) * s).
    // Then,
    //     T = sqrt(P(s).x^2 + P(s).y^2)
    //   T^2 = (x_0 + (x_1 - x_0) * s)^2 + (y_0 + (y_1 - y_0) * s)^2
    //   T^2 = x_0^2 + 2x_0(x_1-x_0)s + (x_1-x_0)^2*s^2
    //       + y_0^2 + 2y_0(y_1-y_0)s + (y_1-y_0)^2*s^2
    //   T^2 = x_0^2 + 2x_0x_1s - 2x_0^2*s + x_1^2*s^2 - 2x_0x_1s^2 + x_0^2*s^2
    //       + y_0^2 + 2y_0y_1s - 2y_0^2*s + y_1^2*s^2 - 2y_0y_1s^2 + y_0^2*s^2
    //     0 = (x_0^2 + y_0^2 + x_1^2 + y_1^2 - 2x_0x_1 - 2y_0y_1)s^2
    //       + (2x_0x_1 + 2y_0y_1 - 2x_0^2 - 2y_0^2)s
    //       + (x_0^2 + y_0^2 - T^2).
    //
    // To simplify, we can factor out some common parts:
    // Let l_0 = x_0^2 + y_0^2, l_1 = x_1^2 + y_1^2, and
    // p = x_0 * x_1 + y_0 * y_1.
    // Then we have
    //   0 = (l_0 + l_1 - 2p)s^2 + 2(p - l_0)s + (l_0 - T^2),
    // with which we can solve for s using the quadratic formula.

    double l_0 = x_0 * x_0 + y_0 * y_0;
    double l_1 = x_1 * x_1 + y_1 * y_1;
    double sqrt_l_0 = Math.sqrt(l_0);
    double diff = Math.sqrt(l_1) - sqrt_l_0;
    if (Math.abs(diff) <= maxVelStep) {
      // Can go all the way to s=1.
      return 1.0;
    }

    double target = sqrt_l_0 + Math.copySign(maxVelStep, diff);
    double p = x_0 * x_1 + y_0 * y_1;

    // Quadratic of s
    double a = l_0 + l_1 - 2 * p;
    double b = 2 * (p - l_0);
    double c = l_0 - target * target;
    double root = Math.sqrt(b * b - 4 * a * c);

    // Check if either of the solutions are valid
    // Won't divide by zero because it is only possible for a to be zero if the
    // target velocity is exactly the same or the reverse of the current
    // velocity, which would be caught by the difference check.
    double s_1 = (-b + root) / (2 * a);
    if (isValidS(s_1)) {
      return s_1;
    }
    double s_2 = (-b - root) / (2 * a);
    if (isValidS(s_2)) {
      return s_2;
    }

    // Since we passed the initial max_vel_step check, a solution should exist,
    // but if no solution was found anyway, just don't limit movement
    return 1.0;
  }

  static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  static boolean epsilonEquals(ChassisSpeeds s1, ChassisSpeeds s2) {
    return epsilonEquals(s1.vxMetersPerSecond, s2.vxMetersPerSecond)
        && epsilonEquals(s1.vyMetersPerSecond, s2.vyMetersPerSecond)
        && epsilonEquals(s1.omegaRadiansPerSecond, s2.omegaRadiansPerSecond);
  }

  /**
   * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
   * direction).
   *
   * @param prevToGoal The rotation from the previous state to the goal state (i.e.
   *     prev.inverse().rotateBy(goal)).
   * @return True if the shortest path to achieve this rotation involves flipping the drive
   *     direction.
   */
  static boolean shouldFlipHeading(double prevToGoal) {
    return Math.abs(prevToGoal) > Math.PI / 2.0;
  }

  /**
   * @see Rotation2d#rotateBy(Rotation2d)
   */
  static double rotateBy(double rad, double otherCos, double otherSin) {
    return Math.atan2(
        Math.sin(rad) * otherCos + Math.cos(rad) * otherSin,
        Math.cos(rad) * otherCos - Math.sin(rad) * otherSin);
  }

  static double rotateBy(double rad, Rotation2d other) {
    return rotateBy(rad, other.getCos(), other.getSin());
  }

  static double angleOf(Translation2d vec) {
    return Math.atan2(vec.getY(), vec.getX());
  }

  static double angularDifference(double prevRadians, double desiredRads) {
    // this looks messy without using Rotation2d methods.
    // this is roughly equivalent to:
    //
    // double r = vars.prev[m].rot2d().unaryMinus()
    //    .rotateBy(vars.desired[m].rot2d()).getRadians();
    double unaryMinusPrevRads = -prevRadians;
    return rotateBy(unaryMinusPrevRads, Math.cos(desiredRads), Math.sin(desiredRads));
  }

  static final class LocalVectors implements StructSerializable {
    public double vx, vy, cos, sin;

    public LocalVectors() {}

    public void reset() {
      vx = vy = cos = sin = 0.0;
    }

    public void applyModuleState(SwerveModuleState state) {
      cos = state.angle.getCos();
      sin = state.angle.getSin();
      vx = cos * state.speedMetersPerSecond;
      vy = sin * state.speedMetersPerSecond;
      if (state.speedMetersPerSecond < 0.0) {
        applyRotation(Rotation2d.k180deg.getCos(), Rotation2d.k180deg.getSin());
      }
    }

    public LocalVectors applyRotation(double otherCos, double otherSin) {
      double newCos = cos * otherCos - sin * otherSin;
      double newSin = cos * otherSin + sin * otherCos;
      cos = newCos;
      sin = newSin;

      return this;
    }

    public double radians() {
      return Math.atan2(sin, cos);
    }

  }

  static final class LocalVars implements StructSerializable {
    public LocalVectors[] prev;

    public LocalVectors[] desired;

    public boolean needToSteer = true;
    public double minS, dt;
    public double dx, dy, dtheta;
    public ChassisSpeeds prevSpeeds, desiredSpeeds;
    public double inputVoltage;
    public Optional<ChassisConstraints> constraintsOpt;

    public SwerveModuleState[] prevModuleStates;

    public SwerveModuleState[] desiredModuleStates;

    public Rotation2d[] steeringOverride;

    public LocalVars() {
      desiredSpeeds = prevSpeeds = new ChassisSpeeds();
      prev = new LocalVectors[NUM_MODULES];
      desired = new LocalVectors[NUM_MODULES];
      steeringOverride = new Rotation2d[NUM_MODULES];
      for (int i = 0; i < NUM_MODULES; i++) {
        prev[i] = new LocalVectors();
        desired[i] = new LocalVectors();
      }
    }

    public LocalVars reset() {
      needToSteer = true;
      Arrays.fill(steeringOverride, null);
      for (int i = 0; i < NUM_MODULES; i++) {
        prev[i].reset();
        desired[i].reset();
      }
      minS = 1.0;
      dx = dy = dtheta = 0.0;

      return this;
    }
  }
}
