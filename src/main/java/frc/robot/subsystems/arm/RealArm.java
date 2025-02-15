package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.wpilibj2.command.Commands.defer;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.elevator.RealElevatorS;
import frc.robot.subsystems.arm.elevator.RealElevatorS.ElevatorConstants;
import frc.robot.subsystems.arm.pivot.MainPivotS;
import frc.robot.subsystems.arm.wrist.NoneWristS;
import frc.robot.subsystems.arm.wrist.Wrist;
import frc.robot.subsystems.arm.wrist.RealWristS;
import frc.robot.subsystems.arm.wrist.RealWristS.WristConstants;

import java.util.Set;

@Logged
public class RealArm extends Arm {
  public void update() {
    position =
        new ArmPosition(
            Radians.of(mainPivotS.getAngleRadians()),
            Meters.of(elevatorS.getLengthMeters()),
            Radians.of(wristS.getAngleRadians()));
  }

  public ArmPosition getPosition() {
    return position;
  }
  @Override
  public MechanismLigament2d getMechanism() {
    return ARM;
  }

  public RealArm() {
    mainPivotS.setLengthSupplier(elevatorS::getLengthMeters);
    mainPivotS.setMoISupplier(elevatorS::getMoI);
    elevatorS.setAngleSupplier(mainPivotS::getAngleRadians);
    wristS.setMainAngleSupplier(mainPivotS::getAngleRadians);
    ARM = mainPivotS.MAIN_PIVOT;
    ARM.append(elevatorS.ELEVATOR);
    elevatorS.ELEVATOR.append(wristS.WRIST);
    
    wristS.WRIST.append(new MechanismLigament2d("hand", 0.3, 0));
  }

  public MainPivotS mainPivotS = new MainPivotS();
  public RealElevatorS elevatorS = new RealElevatorS();
  public RealWristS wristS = new RealWristS();
  private static final Distance SAFE_PIVOT_ELEVATOR_LENGTH =
  ArmPosition.SAFE_PIVOT_ELEVATOR_LENGTH;
  private static final Distance MIN_ELEVATOR_LENGTH = ElevatorConstants.MIN_PADDED_LENGTH;
  private static final Angle SAFE_WRIST_MIN = WristConstants.CW_LIMIT;
  private static final Angle SAFE_WRIST_MAX = WristConstants.CCW_LIMIT;
  public Command goToPosition(ArmPosition position) {
    return defer(
        () -> {
          double startMainPivot = mainPivotS.getAngleRadians();
          double startElevator = elevatorS.getLengthMeters();
          double startWrist = wristS.getAngleRadians();
          double dPivot = position.pivotRadians() - startMainPivot;
          double dElevator = position.elevatorMeters() - startElevator;
          double dWrist = position.wristRadians() - startWrist;
          
          // Just need to move wrist
          if (Math.abs(dPivot) < Units.degreesToRadians(4)) {
            return goDirectlyTo(
                position.pivotRadians(), position.elevatorMeters(), position.wristRadians());
          } else {
            double prePivotElevator = MathUtil.clamp(startElevator, MIN_ELEVATOR_LENGTH.in(Meters), SAFE_PIVOT_ELEVATOR_LENGTH.in(Meters));
            double postPivotElevator =MathUtil.clamp(position.elevatorMeters(), MIN_ELEVATOR_LENGTH.in(Meters), SAFE_PIVOT_ELEVATOR_LENGTH.in(Meters));
            double retractWrist = MathUtil.clamp(startWrist, SAFE_WRIST_MIN.in(Radians), SAFE_WRIST_MAX.in(Radians));
            double retractWristTarget = MathUtil.clamp(position.wristRadians(), SAFE_WRIST_MIN.in(Radians), SAFE_WRIST_MAX.in(Radians));
            return sequence(
                // retract wrist
                goDirectlyTo(startMainPivot, startElevator, retractWristTarget)
                .until(
                        () ->
                        wristS.getAngleRadians() > SAFE_WRIST_MIN.in(Radians) && wristS.getAngleRadians() < SAFE_WRIST_MAX.in(Radians)),
                // Retract elevator
                goDirectlyTo(startMainPivot, prePivotElevator, retractWristTarget)
                    .until(
                        () ->
                            Math.abs(elevatorS.getLengthMeters() - prePivotElevator)
                                < Units.inchesToMeters(1)),
                // pivot
                goDirectlyTo(position.pivotRadians(), postPivotElevator, retractWristTarget)
                    .until(
                        () ->
                            Math.abs(elevatorS.getLengthMeters() - postPivotElevator)
                                < Units.inchesToMeters(1)
                            && Math.abs(mainPivotS.getAngleRadians() - position.pivotRadians()) < Units.degreesToRadians(10)),
                // extend
                goDirectlyTo(
                    position.pivotRadians(), position.elevatorMeters(), retractWristTarget)
                    .until(
                      () ->
                          Math.abs(elevatorS.getLengthMeters() - position.elevatorMeters())
                              < Units.inchesToMeters(1)),
                goDirectlyTo(
                  position.pivotRadians(), position.elevatorMeters(), position.wristRadians())
                    );
          }
        },
        Set.of(mainPivotS, elevatorS, wristS));
  }

  private Command goDirectlyTo(
      double mainPivotRadians, double elevatorMeters, double wristRadians) {
    return parallel(
        mainPivotS.goTo(() -> mainPivotRadians), elevatorS.goToLength(() -> elevatorMeters), wristS.goTo(()->wristRadians));
  }
}
