package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.Commands.defer;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.Set;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.elevator.RealElevatorS;
import frc.robot.subsystems.arm.elevator.RealElevatorS.ElevatorConstants;
import frc.robot.subsystems.arm.pivot.MainPivotS;
import frc.robot.subsystems.arm.pivot.MainPivotS.MainPivotConstants;
import frc.robot.subsystems.arm.wrist.RealWristS;
import frc.robot.subsystems.arm.wrist.RealWristS.WristConstants;

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
    var startTime = Timer.getFPGATimestamp();
    var command = defer(
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
                // TODO: if position is unsafe to fully retract, move to safe position first
                // Retract elevator
                goDirectlyTo(startMainPivot, postPivotElevator, retractWristTarget)
                    .until(
                        () ->
                            elevatorS.getLengthMeters() < SAFE_PIVOT_ELEVATOR_LENGTH.in(Meters) + Units.inchesToMeters(1)),
                // pivot
                goDirectlyTo(position.pivotRadians(), postPivotElevator, retractWristTarget)
                    .until(
                        () ->
                            // Math.abs(elevatorS.getLengthMeters() - postPivotElevator)
                            //     < Units.inchesToMeters(1)
                            //&& 
                            Math.abs(mainPivotS.getAngleRadians() - position.pivotRadians()) < Units.degreesToRadians(10)),
                // extend
                // goDirectlyTo(
                //     position.pivotRadians(), position.elevatorMeters(), retractWristTarget)
                //     .until(
                //       () ->
                //           Math.abs(elevatorS.getLengthMeters() - position.elevatorMeters())
                //               < Units.inchesToMeters(1)),
                goDirectlyTo(
                  position.pivotRadians(), position.elevatorMeters(), position.wristRadians())
                    );
          }
        },
        Set.of(mainPivotS, elevatorS, wristS));
    var endTime = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("goToPositionTime", endTime-startTime);
    return command;
  }
  
  private Command goDirectlyTo(
      double mainPivotRadians, double elevatorMeters, double wristRadians) {
    return parallel(
        mainPivotS.goTo(() -> mainPivotRadians),
        elevatorS.goToLength(
          ()-> {
            var dontHitDrivetrainTarget = (mainPivotS.getAngleRadians() < Units.degreesToRadians(30)) ? 
            Math.max(elevatorMeters, Arm.Positions.GROUND_ALGAE.elevatorMeters()) : elevatorMeters;
            // don't put the wrist axis more than a few inches outside fp
            return MathUtil.clamp(
              dontHitDrivetrainTarget,
              ElevatorConstants.MIN_LENGTH.in(Meters),
              
              ElevatorConstants.MAX_LENGTH.in(Meters));
              // ,
              // Units.inchesToMeters(35)/Math.cos(mainPivotS.getAngleRadians())));
          }
          //() -> Math.min(elevatorMeters,Units.inchesToMeters(29)/Math.cos(mainPivotS.getAngleRadians()))
        ),
        
        wristS.goTo(
          ()->mainPivotS.getAngleRadians() < Units.degreesToRadians(30) ? Math.min(wristRadians, Units.degreesToRadians(-30)) : wristRadians
          ));
  }

  @Override
  public Command Climb() {
    return mainPivotS.voltage(()->-2).until(this::readyToClimb).andThen(mainPivotS.hold());

  }

  public boolean readyToClimb(){
    return position.mainPivotAngle().lt(MainPivotConstants.climbAngle);
  }

  @Override
  public Command algaeStowWithHome() {
    return sequence(
      goToPosition(Arm.Positions.STOW)
      //.until(
      //  ()->this.position.withinTolerance(Arm.Positions.STOW, Units.degreesToRadians(2), Units.inchesToMeters(0.5), Units.degreesToRadians(360)
      //)),
      // new ScheduleCommand(
      //   wristS.driveToHome()
      // )
    );
  }

}
