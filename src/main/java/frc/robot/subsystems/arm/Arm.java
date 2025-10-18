package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
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
import frc.robot.subsystems.arm.wrist.RealWristS.WristConstants;

public abstract class Arm {
  public MechanismLigament2d ARM;

  public record ArmPosition(Angle mainPivotAngle, Distance elevatorLength, Angle wristAngle) {

    public static final Distance SAFE_PIVOT_ELEVATOR_LENGTH = ElevatorConstants.MIN_LENGTH.plus(Inches.of(4));
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
          WristConstants.K_G_ANGLE_WITH_CORAL);
    }
  };

  public class Positions {
    // NEW HAND
    public static final ArmPosition GROUND_CORAL = new ArmPosition(Degrees.of(1), ElevatorConstants.MIN_LENGTH,
        Degrees.of(-46));
    public static final ArmPosition GROUND_CORAL_L1 = new ArmPosition(Degrees.of(1), ElevatorConstants.MIN_LENGTH,
        Degrees.of(-46));
    public static final ArmPosition GROUND_ALGAE = new ArmPosition(Degrees.of(6.7), Meters.of(0.75),
        Degrees.of(-50));
    public static final ArmPosition WALL_INTAKE_CORAL = new ArmPosition(Degrees.of(69.4), Meters.of(0.814),
        Degrees.of(-24));

    public static final ArmPosition POST_INTAKE_CORAL = new ArmPosition(Degrees.of(85),
        ElevatorConstants.MIN_PADDED_LENGTH.plus(Inches.of(0)), Degrees.of(5));
    // public static final ArmPosition L4_CLOSE = new ArmPosition(Degrees.of(86),
    // ElevatorConstants.MAX_LENGTH, Degrees.of(135));
    public static final ArmPosition L4 = new ArmPosition(Degrees.of(96.5), ElevatorConstants.MAX_LENGTH,
        Degrees.of(100));
    public static final ArmPosition L3 = new ArmPosition(Degrees.of(88), Meters.of(1.025).minus(Inches.of(6)),
        Degrees.of(90 + 35));
    public static final ArmPosition L2 = new ArmPosition(Degrees.of(90), ElevatorConstants.MIN_LENGTH,
        Degrees.of(90 + 55));
    public static final ArmPosition L1 = new ArmPosition(Degrees.of(50),
        ElevatorConstants.MIN_PADDED_LENGTH.plus(Inches.of(2)),
        Degrees.of(-40));

    public static final ArmPosition L4_OPP = new ArmPosition(Degrees.of(68),
        ElevatorConstants.MAX_LENGTH.minus(Inches.of(0)), Degrees.of(55));
    public static final ArmPosition L3_OPP = new ArmPosition(Degrees.of(50),
        ElevatorConstants.MIN_LENGTH.plus(Inches.of(14)),
        Degrees.of(95));
    public static final ArmPosition L3_HIGH_ALG = new ArmPosition(Degrees.of(45),
        ElevatorConstants.MIN_LENGTH.plus(Inches.of(14)),
        Degrees.of(-70));
    public static final ArmPosition L2_OPP = new ArmPosition(Degrees.of(35),
        ElevatorConstants.MIN_LENGTH.plus(Inches.of(4)),
        Degrees.of(110));

    public static final ArmPosition NEW_CORAL_STOW = new ArmPosition(Degrees.of(88),
        Meters.of(1.025).minus(Inches.of(6)),
        Degrees.of(90 + 35));

    public static final ArmPosition CORAL_STOW = new ArmPosition(Degrees.of(87), ElevatorConstants.MIN_PADDED_LENGTH,
        Degrees.of(80));
    public static final ArmPosition LOW_ALGAE_REEF = new ArmPosition(Degrees.of(108),
        ElevatorConstants.MIN_LENGTH.plus(Inches.of(1.5)), Degrees.of(77));
    public static final ArmPosition LOW_ALGAE = new ArmPosition(Degrees.of(55),
        ElevatorConstants.MIN_LENGTH.plus(Inches.of(8)), Degrees.of(-55));

    public static final ArmPosition HIGH_ALGAE_REEF = new ArmPosition(Degrees.of(96),
        Meters.of(1.025).plus(Inches.of(5)), Degrees.of(90 + 15));
    public static final ArmPosition HIGH_ALGAE = new ArmPosition(Degrees.of(63),
        ElevatorConstants.MIN_LENGTH.plus(Inches.of(22)), Degrees.of(-60));

    public static final ArmPosition STOW = new ArmPosition(Degrees.of(70), Meters.of(0.75), // algae stow
        Radians.of(0));
    public static final ArmPosition PRE_CLIMB = new ArmPosition(Degrees.of(92), ElevatorConstants.MIN_PADDED_LENGTH,
        Degrees.of(-50));
    public static final ArmPosition SCORE_BARGE_VERTICAL = new ArmPosition(Degrees.of(90), ElevatorConstants.MAX_LENGTH,
        Degrees.of(-20));
    public static final ArmPosition SCORE_BARGE = new ArmPosition(Degrees.of(70), ElevatorConstants.MAX_LENGTH,
        Degrees.of(0));
    public static final ArmPosition SCORE_BARGE_PRE = new ArmPosition(Degrees.of(70), Meters.of(0.75),
        Degrees.of(0));
    public static final ArmPosition SCORE_PROCESSOR = new ArmPosition(Degrees.of(35),
        Meters.of(0.75), Degrees.of(-70));

  }

  public Arm() {
  }

  public ArmPosition position;

  public Command processorWithHome() {
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
        position, Units.degreesToRadians(2), Units.inchesToMeters(0.5), Units.degreesToRadians(8));
  }

  public abstract Command Climb();

  public abstract boolean readyToClimb();
}
