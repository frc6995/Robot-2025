package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.elevator.RealElevatorS.ElevatorConstants;
import frc.robot.subsystems.arm.wrist.RealWristS.WristConstants;

public class RobotVisualizer {
  private static final double BASE_X = Units.feetToMeters(3);
  private static final Color8Bit ORANGE = new Color8Bit(235, 137, 52);
  public static final Mechanism2d MECH_VISUALIZER =
      new Mechanism2d(BASE_X * 2, Units.feetToMeters(7));
  private static final MechanismRoot2d MECH_VISUALIZER_ROOT =
      MECH_VISUALIZER.getRoot("root", BASE_X, Units.inchesToMeters(7.5));
  private static final MechanismRoot2d ARM_PIVOT_BASE =
      MECH_VISUALIZER.getRoot(
          "arm-base", BASE_X - Units.inchesToMeters(10), Units.inchesToMeters(11));
  private static final MechanismRoot2d INTAKE_PIVOT_BASE =
      MECH_VISUALIZER.getRoot(
          "algae-intake-pivot-base",
          BASE_X + Units.inchesToMeters(11.5),
          Units.inchesToMeters(9.5));
  private static final MechanismLigament2d BACK_DRIVETRAIN_HALF =
      new MechanismLigament2d("drive-back", Units.inchesToMeters(14), 180, 4, ORANGE);
  private static final MechanismLigament2d FRONT_DRIVETRAIN_HALF =
      new MechanismLigament2d("drive-front", Units.inchesToMeters(14), 0, 4, ORANGE);

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

  final static double PIVOT_X = -Units.inchesToMeters(10);
  final static double PIVOT_Z = Units.inchesToMeters(11);
  final static Pose3d PIVOT_BASE = new Pose3d(PIVOT_X, 0, PIVOT_Z, Rotation3d.kZero);
  private static Pose3d[] components = new Pose3d[] {Pose3d.kZero,Pose3d.kZero,Pose3d.kZero,new Pose3d(new Translation3d(ElevatorConstants.MIN_LENGTH.in(Meters), 0,0), new Rotation3d(0,0,0))};
  public static Pose3d[] getComponents() {return components;}
  public static void setArmPosition(ArmPosition position) {
    components[0] = PIVOT_BASE.transformBy(new Transform3d(Translation3d.kZero, new Rotation3d(0,-position.pivotRadians(),0)));
    components[1] = components[0].transformBy(new Transform3d(new Translation3d(
      (position.elevatorMeters()-ElevatorConstants.MIN_LENGTH.in(Meters))/2  + Units.inchesToMeters(0), 0,0), Rotation3d.kZero));
    components[2] = components[0].transformBy(new Transform3d(new Translation3d(position.elevatorMeters() - ElevatorConstants.MIN_LENGTH.in(Meters) + Units.inchesToMeters(0), 0,0), Rotation3d.kZero));
    components[3] = components[0].transformBy(new Transform3d(new Translation3d(position.elevatorMeters(), 0,0), new Rotation3d(0,-(position.wristRadians()-Units.degreesToRadians(85)),0)));
  }
}
