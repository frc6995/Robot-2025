package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.Commands.defer;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.elevator.RealElevatorS;
import frc.robot.subsystems.arm.elevator.RealElevatorS.ElevatorConstants;
import frc.robot.subsystems.arm.pivot.MainPivotS;
import frc.robot.subsystems.arm.wrist.NoneWristS;
import frc.robot.subsystems.arm.wrist.Wrist;
import frc.robot.subsystems.arm.wrist.RealWristS;

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
    mainPivotS.setLengthSupplier(()->ElevatorConstants.MIN_LENGTH.in(Meters));//elevatorS::getLengthMeters);
    mainPivotS.setMoISupplier(()->ElevatorConstants.getMoI(ElevatorConstants.MIN_LENGTH.in(Meters)));
    elevatorS.setAngleSupplier(mainPivotS::getAngleRadians);
    ARM = mainPivotS.MAIN_PIVOT;
    ARM.append(elevatorS.ELEVATOR);
    elevatorS.ELEVATOR.append(wristS.WRIST);
    
    wristS.WRIST.append(new MechanismLigament2d("hand", 0.3, 0));
  }

  public MainPivotS mainPivotS = new MainPivotS();
  public RealElevatorS elevatorS = new RealElevatorS();
  public RealWristS wristS = new RealWristS();
  private static final Distance SAFE_PIVOT_ELEVATOR_LENGTH =
      ElevatorConstants.MIN_LENGTH.plus(Inches.of(3));

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
          if (Math.abs(dPivot) < Units.degreesToRadians(1)) {
            return goDirectlyTo(
                position.pivotRadians(), position.elevatorMeters(), position.wristRadians());
          } else {
            double prePivotElevator =
                Math.min(startElevator, SAFE_PIVOT_ELEVATOR_LENGTH.in(Meters));
            double postPivotElevator =
                Math.min(position.elevatorMeters(), SAFE_PIVOT_ELEVATOR_LENGTH.in(Meters));
            return sequence(
                // Retract elevator
                goDirectlyTo(startMainPivot, prePivotElevator, startWrist)
                    .until(
                        () ->
                            Math.abs(elevatorS.getLengthMeters() - prePivotElevator)
                                < Units.inchesToMeters(1)),
                goDirectlyTo(position.pivotRadians(), postPivotElevator, startWrist)
                    .until(
                        () ->
                            Math.abs(elevatorS.getLengthMeters() - postPivotElevator)
                                < Units.inchesToMeters(1)),
                goDirectlyTo(
                    position.pivotRadians(), position.elevatorMeters(), position.wristRadians()));
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
