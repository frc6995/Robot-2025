package frc.robot.trajectories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final Distance ALG_BACK = Meters.of(0.254);
    public static final Distance ALG_DIST = Meters.of(1.554);
    public static final LinearVelocity APPROACH = MetersPerSecond.of(1.25);
    public static final Angle A_CS = Radians.of(0.628);
    public static final Distance BACKUP = Meters.of(0.165);
    public static final Distance CS_BACKUP = Meters.of(0.114);
    public static final Distance FieldWidth = Meters.of(8.052);
    public static final double L1 = 8.14;
    public static final double L1p = 7.123;
    public static final double L2 = 6.75;
    public static final double L2p = 5.906;
    public static final double L3 = 6.12;
    public static final double L3p = 5.355;
    public static final Distance LP_SHORT = Meters.of(0.102);
    public static final LinearVelocity MAX_INTAKE_VEL = MetersPerSecond.of(3.5);
    public static final LinearVelocity MAX_LP_VEL = MetersPerSecond.of(0.5);
    public static final LinearVelocity MAX_START_VEL = MetersPerSecond.of(2.5);
    public static final LinearVelocity MAX_VEL = MetersPerSecond.of(2.7);

    public static final class Poses {
        public static final Pose2d A = new Pose2d(3.023, 4.185, Rotation2d.fromRadians(3.142));
        public static final Pose2d ALG = new Pose2d(7.53, 5.203, Rotation2d.kZero);
        public static final Pose2d B = new Pose2d(3.023, 3.867, Rotation2d.fromRadians(3.142));
        public static final Pose2d C = new Pose2d(3.62, 2.843, Rotation2d.fromRadians(4.189));
        public static final Pose2d CL1 = new Pose2d(8, 7.301, Rotation2d.fromRadians(3.142));
        public static final Pose2d CL2 = new Pose2d(8, 6.189, Rotation2d.fromRadians(3.142));
        public static final Pose2d CL3 = new Pose2d(8, 5.1, Rotation2d.fromRadians(3.142));
        public static final Pose2d D = new Pose2d(3.9, 2.678, Rotation2d.fromRadians(4.189));
        public static final Pose2d E = new Pose2d(5.084, 2.678, Rotation2d.fromRadians(5.236));
        public static final Pose2d F = new Pose2d(5.363, 2.843, Rotation2d.fromRadians(5.236));
        public static final Pose2d G = new Pose2d(5.961, 3.867, Rotation2d.kZero);
        public static final Pose2d H = new Pose2d(5.961, 4.185, Rotation2d.kZero);
        public static final Pose2d I = new Pose2d(5.363, 5.209, Rotation2d.fromRadians(-5.236));
        public static final Pose2d J = new Pose2d(5.084, 5.374, Rotation2d.fromRadians(1.047));
        public static final Pose2d K = new Pose2d(3.9, 5.374, Rotation2d.fromRadians(2.094));
        public static final Pose2d L = new Pose2d(3.62, 5.21, Rotation2d.fromRadians(2.094));
        public static final Pose2d L1_A = new Pose2d(3, 4.003, Rotation2d.fromRadians(0.524));
        public static final Pose2d L1_B = new Pose2d(3, 4.05, Rotation2d.fromRadians(-0.524));
        public static final Pose2d L1_L = new Pose2d(3.552, 5.269, Rotation2d.fromRadians(-1.493));
        public static final Pose2d L2_A = new Pose2d(3.023, 4.185, Rotation2d.fromRadians(6.283));
        public static final Pose2d L2_B = new Pose2d(3.023, 3.867, Rotation2d.fromRadians(6.283));
        public static final Pose2d LP1 = new Pose2d(1.376, 5.73, Rotation2d.fromRadians(2.404));
        public static final Pose2d LP2 = new Pose2d(1.396, 4.061, Rotation2d.fromRadians(-2.793));
        public static final Pose2d LP2_APP = new Pose2d(2.141, 4.323, Rotation2d.fromRadians(-2.793));
        public static final Pose2d LP3 = new Pose2d(1.376, 2.322, Rotation2d.fromRadians(-2.404));
        public static final Pose2d M2_SL3_C = new Pose2d(2.223, 6.145, Rotation2d.kZero);
        public static final Pose2d M_SL3_C = new Pose2d(2.763, 3.378, Rotation2d.kZero);
        public static final Pose2d M_SL3_H = new Pose2d(5.788, 5.263, Rotation2d.kZero);
        public static final Pose2d PROC = new Pose2d(5.8, 0.7, Rotation2d.fromRadians(-1.309));
        public static final Pose2d R1 = new Pose2d(2.938, 4.026, Rotation2d.kZero);
        public static final Pose2d R2 = new Pose2d(3.715, 2.681, Rotation2d.fromRadians(1.047));
        public static final Pose2d R3 = new Pose2d(5.269, 2.681, Rotation2d.fromRadians(2.094));
        public static final Pose2d R4 = new Pose2d(6.046, 4.026, Rotation2d.fromRadians(3.142));
        public static final Pose2d R5 = new Pose2d(5.269, 5.372, Rotation2d.fromRadians(4.189));
        public static final Pose2d R6 = new Pose2d(3.715, 5.372, Rotation2d.fromRadians(5.236));
        public static final Pose2d REEF = new Pose2d(4.492, 4.026, Rotation2d.kZero);
        public static final Pose2d SL1 = new Pose2d(0.625, 6.694, Rotation2d.fromRadians(2.199));
        public static final Pose2d SL2 = new Pose2d(1.211, 7.129, Rotation2d.fromRadians(2.199));
        public static final Pose2d SL3 = new Pose2d(1.672, 7.322, Rotation2d.fromRadians(2.199));
        public static final Pose2d SR1 = new Pose2d(0.625, 1.358, Rotation2d.fromRadians(-2.199));
        public static final Pose2d SR2 = new Pose2d(1.211, 0.923, Rotation2d.fromRadians(-2.199));
        public static final Pose2d SR3 = new Pose2d(1.672, 0.73, Rotation2d.fromRadians(-2.199));
        public static final Pose2d STA = new Pose2d(7.107, 6.209, Rotation2d.fromRadians(0.698));
        public static final Pose2d STAP = new Pose2d(7.132, 6.209, Rotation2d.fromRadians(1.571));
        public static final Pose2d STAW = new Pose2d(7.133, 7.587, Rotation2d.fromRadians(1.571));
        public static final Pose2d STC = new Pose2d(7.107, 1.843, Rotation2d.fromRadians(-0.698));
        public static final Pose2d STCW = new Pose2d(7.133, 0.465, Rotation2d.fromRadians(-1.571));
        public static final Pose2d STE = new Pose2d(7.1, 2.678, Rotation2d.fromRadians(5.236));
        public static final Pose2d STF = new Pose2d(7.2, 2.843, Rotation2d.kZero);
        public static final Pose2d STG = new Pose2d(7.2, 3.867, Rotation2d.kZero);
        public static final Pose2d STH = new Pose2d(7.2, 4.185, Rotation2d.kZero);
        public static final Pose2d STI = new Pose2d(7.2, 5.209, Rotation2d.kZero);
        public static final Pose2d STJ = new Pose2d(7.1, 5.374, Rotation2d.fromRadians(1.047));
        public static final Pose2d STL = new Pose2d(7.107, 6.209, Rotation2d.fromRadians(0.698));
        public static final Pose2d STLW = new Pose2d(7.133, 7.587, Rotation2d.fromRadians(1.571));

        private Poses() {}
    }

    private ChoreoVars() {}
}