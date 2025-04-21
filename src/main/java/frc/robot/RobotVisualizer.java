package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeS.HandConstants;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.elevator.RealElevatorS.ElevatorConstants;
import frc.robot.util.Capture;

/**
 * This class manages a Mechanism2d visualizer of the robot and the list of
 * Pose3ds for AdvantageScope 3d field components.
 * 
 * Base model: drivetrain, pivot, and pivot chain
 * Component 0: base elevator stage and big sprocket
 * Component 1: middle elevator stage
 * Component 2: Top elevator stage and fixed pulley for wrist
 * Component 3: end effector.
 */
public class RobotVisualizer {
    private static final double BASE_X = Units.feetToMeters(3);
    private static final Color8Bit ORANGE = new Color8Bit(235, 137, 52);
    public static final Mechanism2d MECH_VISUALIZER = new Mechanism2d(BASE_X * 2, Units.feetToMeters(7));
    private static final MechanismRoot2d MECH_VISUALIZER_ROOT = MECH_VISUALIZER.getRoot("root", BASE_X,
            Units.inchesToMeters(7.5));
    private static final MechanismRoot2d ARM_PIVOT_BASE = MECH_VISUALIZER.getRoot(
            "arm-base", BASE_X - Units.inchesToMeters(10), Units.inchesToMeters(11));
    private static final MechanismRoot2d INTAKE_PIVOT_BASE = MECH_VISUALIZER.getRoot(
            "algae-intake-pivot-base",
            BASE_X + Units.inchesToMeters(11.5),
            Units.inchesToMeters(9.5));
    private static final MechanismLigament2d BACK_DRIVETRAIN_HALF = new MechanismLigament2d("drive-back",
            Units.inchesToMeters(14), 180, 4, ORANGE);
    private static final MechanismLigament2d FRONT_DRIVETRAIN_HALF = new MechanismLigament2d("drive-front",
            Units.inchesToMeters(14), 0, 4, ORANGE);

    public static void setupVisualizer() {
        MECH_VISUALIZER_ROOT.append(BACK_DRIVETRAIN_HALF);
        MECH_VISUALIZER_ROOT.append(FRONT_DRIVETRAIN_HALF);
    }

    public static void addAlgaeIntake(MechanismLigament2d intake) {
        INTAKE_PIVOT_BASE.append(intake);
    }

    public static void addArmPivot(MechanismLigament2d pivot) {
        ARM_PIVOT_BASE.append(pivot);
    }

    static final double PIVOT_X = -Units.inchesToMeters(10);
    static final double PIVOT_Z = Units.inchesToMeters(11);
    static final Pose3d PIVOT_BASE = new Pose3d(PIVOT_X, 0, PIVOT_Z, Rotation3d.kZero);
    private static Pose3d[] components = new Pose3d[] {
            Pose3d.kZero,
            Pose3d.kZero,
            Pose3d.kZero,
            new Pose3d(
                    new Translation3d(ElevatorConstants.MIN_LENGTH.in(Meters), 0, 0),
                    new Rotation3d(0, 0, 0))
    };

    public static Pose3d[] getComponents() {
        return components;
    }

    public static void setArmPosition(ArmPosition position) {
        components[0] = PIVOT_BASE.transformBy(
                new Transform3d(Translation3d.kZero, new Rotation3d(0, -position.pivotRadians(), 0)));
        components[1] = components[0].transformBy(
                new Transform3d(
                        new Translation3d(
                                (position.elevatorMeters() - ElevatorConstants.MIN_LENGTH.in(Meters)) / 2
                                        + Units.inchesToMeters(0),
                                0,
                                0),
                        Rotation3d.kZero));
        components[2] = components[0].transformBy(
                new Transform3d(
                        new Translation3d(
                                position.elevatorMeters()
                                        - ElevatorConstants.MIN_LENGTH.in(Meters)
                                        + Units.inchesToMeters(0),
                                0,
                                0),
                        Rotation3d.kZero));
        components[3] = components[0].transformBy(
                new Transform3d(
                        new Translation3d(position.elevatorMeters(), 0, 0),
                        new Rotation3d(0, -(position.wristRadians()), 0)));
    }

    static Capture<Pose3d> lastCoralPose = new Capture<Pose3d>(Pose3d.kZero);

    private static Pose3d getIntakePose(Pose2d drivebasePose) {
        return new Pose3d(drivebasePose)
                // position of
                .plus(
                        new Transform3d(
                                RobotVisualizer.getComponents()[3].getTranslation(),
                                RobotVisualizer.getComponents()[3].getRotation()));
    }

    /**
     * Get the field position of the coral in the hand
     * 
     * @param drivebasePose field drivetrain pose
     * @param coralOffset   Position in meters of the coral in the hand.
     *                      Zero point here was originally when the coral first
     *                      trips the TOF when intaking,
     *                      but no longer matches sensor position and was never
     *                      worth fixing.
     */
    private static Pose3d getCoralPoseIfInHand(Pose2d drivebasePose, double coralOffset) {
        return getIntakePose(drivebasePose)
                .plus(
                        new Transform3d(
                                HandConstants.CORAL_LENGTH_METERS / 2.0
                                        - Units.inchesToMeters(0.9)
                                        - coralOffset,
                                0.0,
                                -Units.inchesToMeters(9.978 - 0.25),
                                Rotation3d.kZero));
    }

    /**
     * Get the field position of the coral in the hand
     * 
     * @param drivebasePose field drivetrain pose
     * @param coralOffset   Position in meters of the coral in the hand.
     *                      Zero point here was originally when the coral first
     *                      trips the TOF when intaking,
     *                      but no longer matches sensor position and was never
     *                      worth fixing.
     * @param hasCoral      if false, this method returns the coral pose the last
     *                      time the coral exited the hand.
     */
    public static Pose3d getCoralPose(Pose2d drivebasePose, double coralOffset, boolean hasCoral) {
        if (hasCoral) {
            return getCoralPoseIfInHand(drivebasePose, coralOffset);
        } else {
            return lastCoralPose.inner;
        }
    }

    public static Pose3d getAlgaePose(Pose2d drivebasePose) {
        return getIntakePose(drivebasePose)
                .plus(new Transform3d(0.42, 0, -0.02, Rotation3d.kZero));
    }

    public static Command markCoralLeavesHand(Supplier<Pose2d> drivebasePose, DoubleSupplier coralOffset) {
        return Commands.runOnce(
                () -> {
                    RobotVisualizer.lastCoralPose.inner = RobotVisualizer.getCoralPoseIfInHand(drivebasePose.get(),
                            coralOffset.getAsDouble());
                })
                .ignoringDisable(true);
    }
}
