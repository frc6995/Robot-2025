package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.elevator.RealElevatorS.ElevatorConstants;
import frc.robot.subsystems.arm.pivot.MainPivotS.MainPivotConstants;

public abstract class Arm {
  public MechanismLigament2d ARM;

  public record ArmPosition(Angle mainPivotAngle, Distance elevatorLength, Angle wristAngle) {
    public double pivotRadians() {
      return mainPivotAngle.in(Radians);
    }

    public double elevatorMeters() {
      return elevatorLength.in(Meters);
    }

    public double wristRadians() {
      return wristAngle.in(Radians);
    }

    public boolean withinTolerance(
        ArmPosition other, double pivotRadians, double elevatorLength, double wristRadians) {
      return MathUtil.isNear(this.pivotRadians(), other.pivotRadians(), pivotRadians)
          && MathUtil.isNear(this.elevatorMeters(), other.elevatorMeters(), elevatorLength)
          && MathUtil.isNear(this.wristRadians(), other.wristRadians(), wristRadians);
    }
  }
  ;

  public class Positions {
    public static final ArmPosition L2 =
    new ArmPosition(Degrees.of(95), ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(0));
    public static final ArmPosition L3 =
        new ArmPosition(Degrees.of(95), Meters.of(1), Radians.of(0));
    public static final ArmPosition L4 =
        new ArmPosition(Degrees.of(95), ElevatorConstants.MAX_LENGTH, Radians.of(0));
    public static final ArmPosition STOW =
        new ArmPosition(MainPivotConstants.CW_LIMIT, ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(2));
    public static final ArmPosition INTAKE_CORAL =
        new ArmPosition(MainPivotConstants.CW_LIMIT.plus(Degrees.of(3)), ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(0.3));
    public static final ArmPosition LOW_ALGAE = new ArmPosition(MainPivotConstants.CW_LIMIT, ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(0));
    public static final ArmPosition HIGH_ALGAE = new ArmPosition(MainPivotConstants.CW_LIMIT, ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(0));
    public static final ArmPosition GROUND_ALGAE = new ArmPosition(MainPivotConstants.CW_LIMIT, ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(0));
    public static final ArmPosition SCORE_BARGE = new ArmPosition(Degrees.of(95), ElevatorConstants.MAX_LENGTH, Radians.of(0));
    public static final ArmPosition SCORE_PROCESSOR = new ArmPosition(MainPivotConstants.CW_LIMIT, ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(0));
    public static final ArmPosition PRE_CLIMB = new ArmPosition(Degrees.of(95), ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(0));
    public static final ArmPosition POST_CLIMB = new ArmPosition(MainPivotConstants.CW_LIMIT, ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(0));

  }

  public Arm() {}

  protected ArmPosition position;

  public Command goToPosition(ArmPosition position) {
    return Commands.none();
  }

  public MechanismLigament2d getMechanism() {
    return ARM;
  }

  public abstract void update();

  public boolean atPosition(ArmPosition position) {
    return this.position.withinTolerance(
        position, Units.degreesToRadians(1), Units.inchesToMeters(0.5), Units.degreesToRadians(1));
  }
}
