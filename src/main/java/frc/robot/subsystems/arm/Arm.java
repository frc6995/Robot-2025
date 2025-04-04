package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.elevator.RealElevatorS.ElevatorConstants;
import frc.robot.subsystems.arm.pivot.MainPivotS.MainPivotConstants;
import frc.robot.subsystems.arm.wrist.RealWristS.WristConstants;

public abstract class Arm {
  public MechanismLigament2d ARM;
  

  public record ArmPosition(Angle mainPivotAngle, Distance elevatorLength, Angle wristAngle) {
    public static final Distance SAFE_PIVOT_ELEVATOR_LENGTH =
    ElevatorConstants.MIN_LENGTH.plus(Inches.of(6));
    public static final Angle SAFE_WRIST = WristConstants.CW_LIMIT.plus(Degrees.of(30));
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

    public ArmPosition premove() {
      return new ArmPosition(mainPivotAngle,
        elevatorLength.gt(SAFE_PIVOT_ELEVATOR_LENGTH) ? SAFE_PIVOT_ELEVATOR_LENGTH : elevatorLength,
        wristAngle);
    }

    public ArmPosition safeWrist() {
      return new ArmPosition(mainPivotAngle,
        elevatorLength,
        wristAngle.lt(Rotations.of(0))? wristAngle : Rotations.of(-0.25));
    }
  }
  ;

  public class Positions {
    public static final ArmPosition L1 = 
    new ArmPosition(Degrees.of(35), ElevatorConstants.MIN_PADDED_LENGTH, WristConstants.CW_LIMIT);
    public static final ArmPosition L2 =
    new ArmPosition(Degrees.of(95), ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(0.35));
    public static final ArmPosition L3 =
        new ArmPosition(Degrees.of(90), Meters.of(1.025), Radians.of(0.24));
    public static final ArmPosition L4 =
        new ArmPosition(Degrees.of(90), ElevatorConstants.MAX_LENGTH, Radians.of(0.29));
    public static final ArmPosition STOW =
        new ArmPosition(Degrees.of(70), ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(-0.5));
    public static final ArmPosition CLOSE_INTAKE_CORAL =
        new ArmPosition(Radians.of(0.957), ElevatorConstants.MIN_PADDED_LENGTH, WristConstants.CW_LIMIT);
    public static final ArmPosition INTAKE_CORAL =
        new ArmPosition(Radians.of(0.957), ElevatorConstants.MIN_PADDED_LENGTH.plus(Inches.of(1)), WristConstants.CW_LIMIT);
    public static final ArmPosition WALL_INTAKE_CORAL =
        new ArmPosition(Degrees.of(60.11 - 0.5), ElevatorConstants.MIN_PADDED_LENGTH.plus(Inches.of(0.5)), WristConstants.CW_LIMIT);
    public static final ArmPosition POST_INTAKE_CORAL = new ArmPosition(Degrees.of(85), ElevatorConstants.MIN_PADDED_LENGTH.plus(Inches.of(1)), WristConstants.CW_LIMIT);

    public static final ArmPosition LOW_ALGAE = new ArmPosition(Degrees.of(55), ElevatorConstants.MIN_LENGTH.plus(Inches.of(8)), WristConstants.CW_LIMIT.plus(Degrees.of(40)));
    public static final ArmPosition HIGH_ALGAE = new ArmPosition(Degrees.of(60), ElevatorConstants.MIN_LENGTH.plus(Inches.of(22)), WristConstants.CW_LIMIT.plus(Degrees.of(40)));
    public static final ArmPosition GROUND_ALGAE = new ArmPosition(Degrees.of(17), ElevatorConstants.MIN_PADDED_LENGTH.plus(Inches.of(6)), WristConstants.CW_LIMIT.plus(Degrees.of(7)));
    //public static final ArmPosition SCORE_BARGE = new ArmPosition(Degrees.of(80), ElevatorConstants.MAX_LENGTH, Degrees.of(-42));
    public static final ArmPosition SCORE_BARGE = new ArmPosition(Degrees.of(75), ElevatorConstants.MAX_LENGTH, Degrees.of(-30));
    public static final ArmPosition SCORE_PROCESSOR = new ArmPosition(MainPivotConstants.CW_LIMIT, ElevatorConstants.MIN_PADDED_LENGTH, WristConstants.CW_LIMIT.plus(Degrees.of(20)));
    public static final ArmPosition PRE_SCORE_PROCESSOR = new ArmPosition(MainPivotConstants.CW_LIMIT, ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(0));
    public static final ArmPosition PRE_CLIMB = new ArmPosition(Degrees.of(90), ElevatorConstants.MIN_PADDED_LENGTH, WristConstants.CW_LIMIT);
    public static final ArmPosition POST_CLIMB = new ArmPosition(MainPivotConstants.CW_LIMIT, ElevatorConstants.MIN_PADDED_LENGTH, Rotations.of(-0.096));

    // ATTN: Outside pivot and wrist soft limits + need wrist hardware change
    private static final ArmPosition LOW_ALGAE_REEF = new ArmPosition(Rotations.of(0.307), ElevatorConstants.MIN_LENGTH, Rotations.of(-0.134));


  }

  public Arm() {}

  public ArmPosition position;
  

  public Command algaeStowWithHome() {
    return Commands.none();

  }
  public Command goToPosition(ArmPosition position) {
    return Commands.none();
  }

  public MechanismLigament2d getMechanism() {
    return ARM;
  }

  public abstract void update();

  public ArmPosition getPosition() {
    return this.position;
  }
  public boolean atPosition(ArmPosition position) {
    return this.position.withinTolerance(
        position, Units.degreesToRadians(2), Units.inchesToMeters(0.5), Units.degreesToRadians(5));
  }
  public abstract Command Climb();
  public abstract boolean readyToClimb();
}
