package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.elevator.RealElevatorS;
import frc.robot.subsystems.arm.elevator.RealElevatorS.ElevatorConstants;
import frc.robot.subsystems.arm.pivot.MainPivotS;
import frc.robot.subsystems.arm.pivot.MainPivotS.MainPivotConstants;
import frc.robot.subsystems.arm.wrist.RealWristS;
import frc.robot.subsystems.arm.wrist.RealWristS.WristConstants;

@Logged
public class RealArm extends Arm {
  public void update() {
    position = new ArmPosition(
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
  private static final Distance SAFE_PIVOT_ELEVATOR_LENGTH = ArmPosition.SAFE_PIVOT_ELEVATOR_LENGTH;
  private static final Distance MIN_ELEVATOR_LENGTH = ElevatorConstants.MIN_LENGTH;
  private static final Angle SAFE_WRIST_MIN = WristConstants.CW_LIMIT;
  private static final Angle SAFE_WRIST_MAX = WristConstants.CCW_LIMIT;
  public Trigger elevatorRetractedEnough = new Trigger(
      () -> elevatorS.getLengthMeters() < SAFE_PIVOT_ELEVATOR_LENGTH.in(Meters) + Units.inchesToMeters(10));

  private Command goToPositionWithoutTuckCheck(ArmPosition position) {
    double positionPivotRadians = position.pivotRadians();
    double positionElevatorMeters = position.elevatorMeters();
    double positionWristRadians = position.wristRadians();
    double postPivotElevator = MathUtil.clamp(positionElevatorMeters, MIN_ELEVATOR_LENGTH.in(Meters),
      SAFE_PIVOT_ELEVATOR_LENGTH.in(Meters));
    
    Command command = either(
      goDirectlyTo(
                positionPivotRadians, positionElevatorMeters, positionWristRadians)
      , sequence(
        // Retract elevator
        goDirectlyToPivotHold(postPivotElevator, positionWristRadians)
            .until(elevatorRetractedEnough),
        // pivot
        goDirectlyTo(positionPivotRadians, postPivotElevator, positionWristRadians)
            .until(
                () -> Math.abs(mainPivotS.getAngleRadians() - positionPivotRadians) < Units.degreesToRadians(10)),
        goDirectlyTo(
            positionPivotRadians, positionElevatorMeters, positionWristRadians))
      , ()->
        Math.abs(positionPivotRadians - mainPivotS.getAngleRadians()) < Units.degreesToRadians(6)
      );
    return command;
  }

  public Command goToPosition(ArmPosition position) {
    if (position.pivotRadians() < Units.degreesToRadians(0)) {
      double shrunkenElevator = MathUtil.clamp(position.elevatorMeters(), MIN_ELEVATOR_LENGTH.in(Meters),
      SAFE_PIVOT_ELEVATOR_LENGTH.in(Meters));
      Angle wristSafeToLower = position.wristAngle().lt(Degrees.of(-70)) ? Degrees.of(0) : position.wristAngle();
      return
        goToPositionWithoutTuckCheck(new ArmPosition(Degrees.of(17), Meters.of(shrunkenElevator), wristSafeToLower))
        .until(()->wristS.getAngleRadians() > Units.degreesToRadians(-3))
        .unless(()->wristS.getAngleRadians() > Units.degreesToRadians(-3))
        .andThen(goToPositionWithoutTuckCheck(position));
    } else {
      return goToPositionWithoutTuckCheck(position);
    }
  }

  private Command elevatorDirectlyTo(double elevatorMeters) {
    return elevatorS.goToLength(
            () -> {
              var dontHitDrivetrainTarget = elevatorMeters;
              // don't put the wrist axis more than a few inches outside fp
              return MathUtil.clamp(
                  dontHitDrivetrainTarget,
                  ElevatorConstants.MIN_LENGTH.in(Meters),

                  ElevatorConstants.MAX_LENGTH.in(Meters));
              // ,
              // Units.inchesToMeters(35)/Math.cos(mainPivotS.getAngleRadians())));
            }
        // () ->
        // Math.min(elevatorMeters,Units.inchesToMeters(29)/Math.cos(mainPivotS.getAngleRadians()))
        );
  }
  private Command wristDirectlyTo(double wristRadians, double elevatorSetpointMeters, DoubleSupplier mainPivotRadians) {
    return wristS.goTo(
      () -> {
        double dontGoIntoBumperMinLimit = 
        mainPivotRadians.getAsDouble() < Units.degreesToRadians(0) || mainPivotS.getAngleRadians() < Units.degreesToRadians(0)
          ? 0
          : WristConstants.CW_LIMIT.in(Radians);
        // Only if nearly fully extended can the wrist go to its full limit, since the algae roller touches the tube
        double dontGoIntoElevatorMaxLimit = Math.min(elevatorS.getLengthMeters(), elevatorSetpointMeters) < ElevatorConstants.MAX_LENGTH.in(Meters)-Units.inchesToMeters(6)
        ? WristConstants.CCW_LIMIT.in(Radians)-Units.degreesToRadians(25) : WristConstants.CCW_LIMIT.in(Radians);
        return MathUtil.clamp(wristRadians, dontGoIntoBumperMinLimit, dontGoIntoElevatorMaxLimit);

      }
      );
  }
  private Command goDirectlyTo(
      double mainPivotRadians, double elevatorMeters, double wristRadians) {
    return parallel(
        mainPivotS.goTo(() -> mainPivotRadians),
        elevatorDirectlyTo(elevatorMeters),
        wristDirectlyTo(wristRadians, elevatorMeters, ()->mainPivotRadians));
  }

  private Command goDirectlyToPivotHold(
  
    double elevatorMeters, double wristRadians) {
  return parallel(
      mainPivotS.hold(),
      elevatorDirectlyTo(elevatorMeters),
      wristDirectlyTo(wristRadians, elevatorMeters, mainPivotS::getAngleRadians));
}

  @Override
  public Command Climb() {
    return mainPivotS.voltage(() -> -2).until(this::readyToClimb).andThen(mainPivotS.hold());

  }

  public boolean readyToClimb() {
    return position.mainPivotAngle().lt(MainPivotConstants.climbAngle);
  }

  @Override
  public Command processorWithHome() {

    return sequence(
        goToPosition(Arm.Positions.SCORE_PROCESSOR)
    .until(
    ()->this.position.withinTolerance(Arm.Positions.SCORE_PROCESSOR,
    Units.degreesToRadians(2), Units.inchesToMeters(0.5),
    Units.degreesToRadians(3600)
    )),
    parallel(
    new ScheduleCommand(
    wristS.driveToHome()
    ),
    new ScheduleCommand(
      elevatorDirectlyTo(Arm.Positions.SCORE_PROCESSOR.elevatorMeters())
      ),
      new ScheduleCommand(
        mainPivotS.goTo(Arm.Positions.SCORE_PROCESSOR::pivotRadians)
        )
    ));
  }

}
