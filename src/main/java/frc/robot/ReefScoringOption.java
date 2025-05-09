package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Function;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.elevator.RealElevatorS.ElevatorConstants;
import frc.robot.subsystems.arm.wrist.RealWristS.WristConstants;

/**
 * This enum details the 6 different ways to score coral.
 * Specific to each option is:
 * - a mapping of branch number (0-11) to alignment pose
 * - an arm command to the scoring position (could be a custom sequence)
 * - an outtake voltage for scoring
 * - the actual scoring arm position for reference
 * - an arm command to run after scoring
 * - an arm command to run after intaking/during auto align when not close enough
 * 
 * Since the enum fields are in a static context, the commands are stored as factories which accept the Autos instance.
 */
public enum ReefScoringOption {
  L1(
      POI::selectedL1POI,
      (autos) -> autos.m_arm.goToPosition(Arm.Positions.L1),
      -1.5,
      Arm.Positions.L1,
      (autos) -> autos.m_arm.goToPosition(Arm.Positions.L1),
      (autos) ->
          new ScheduleCommand(
              autos.m_arm.goToPosition(Arm.Positions.L1)) // so tapping the align button goes to L1
      ),
  L2(
      POI::selectedBatterySidePOI,
      (autos) -> autos.m_arm.goToPosition(Arm.Positions.L2_BATT),
      4,
      Arm.Positions.L2_BATT,
      (autos) ->
          autos.m_arm.goToPosition(
              // intentional, we want a pivot up
              new ArmPosition(
                  Arm.Positions.L3_BATT.mainPivotAngle(),
                  ElevatorConstants.MIN_PADDED_LENGTH,
                  Arm.Positions.L2_BATT.wristAngle())),
      (autos) ->
          new ScheduleCommand(
              autos.m_arm.goToPosition(
                  Arm.Positions.L2_BATT)) // so tapping the align button goes to L2
      ),
  L3_BATT(
      POI::selectedBatterySidePOI,
      (autos) -> autos.m_arm.goToPosition(Arm.Positions.L3_BATT),
      4,
      Arm.Positions.L3_BATT,
      (autos) ->
          autos.m_arm.goToPosition(
              new ArmPosition(
                  Arm.Positions.L3_BATT.mainPivotAngle(),
                  ElevatorConstants.MIN_PADDED_LENGTH,
                  Arm.Positions.L3_BATT.wristAngle())),
      (autos) ->
          autos.m_arm.goToPosition(
              new ArmPosition(
                  Arm.Positions.L3_BATT.mainPivotAngle(),
                  ElevatorConstants.MIN_PADDED_LENGTH,
                  Arm.Positions.L3_BATT.wristAngle()))),
  L4_BATT(
      POI::selectedBatterySidePOI,
      (autos) -> autos.m_arm.goToPosition(Arm.Positions.L4_BATT),
      6,
      Arm.Positions.L4_BATT,
      (autos) ->
          autos.m_arm.goToPosition(
              new ArmPosition(
                  Arm.Positions.L4_BATT.mainPivotAngle(),
                  ElevatorConstants.MIN_PADDED_LENGTH,
                  Arm.Positions.L4_BATT.wristAngle())),
      (autos) ->
          autos.m_arm.goToPosition(
              new ArmPosition(
                  Arm.Positions.L4_BATT.mainPivotAngle(),
                  ElevatorConstants.MIN_PADDED_LENGTH,
                  Arm.Positions.L4_BATT.wristAngle()))),
  L3_PIV(
      POI::selectedPivotSidePOI,
      (autos) -> autos.m_arm.goToPosition(Arm.Positions.L3_PIV),
      -4,
      Arm.Positions.L3_PIV,
      (autos) ->
          autos.m_arm.goToPosition(
              new ArmPosition(
                  Arm.Positions.L3_BATT.mainPivotAngle(),
                  ElevatorConstants.MIN_PADDED_LENGTH,
                  Arm.Positions.L3_PIV.wristAngle())),
      (autos) ->
          autos.m_arm.goToPosition(
              new ArmPosition(
                  Arm.Positions.L3_PIV.mainPivotAngle(),
                  ElevatorConstants.MIN_PADDED_LENGTH,
                  Arm.Positions.L3_PIV.wristAngle()))),
  L4_PIV(
      POI::selectedPivotSidePOI,
      (autos) -> autos.m_arm.goToPosition(Arm.Positions.L4_PIV),
      -4,
      Arm.Positions.L4_PIV,
      // Custom stow: Tilt main pivot at least 4 degrees away from the reef (while extended) before retracting.
      // Arm sequencing logic permits the pivot to move a few degrees without retracting.
      (autos) ->
          Commands.sequence(
              autos
                  .m_arm
                  .goToPosition(
                      new ArmPosition(
                          Arm.Positions.L4_PIV.mainPivotAngle().minus(Degrees.of(5)),
                          Arm.Positions.L4_PIV.elevatorLength(),
                          Arm.Positions.L3_PIV.wristAngle()))
                  .until(
                      () ->
                          autos.m_arm.position.pivotRadians()
                              < Arm.Positions.L4_PIV.pivotRadians() - Units.degreesToRadians(4))
                  .withTimeout(1),
              autos.m_arm.goToPosition(
                  new ArmPosition(
                      Arm.Positions.L3_BATT.mainPivotAngle(),
                      ElevatorConstants.MIN_PADDED_LENGTH,
                      Arm.Positions.L3_BATT.wristAngle()))),
      // Premove: Put the wrist in a position that balances its CoG in line with the extending arm to prevent excess torque
      (autos) -> 
          autos.m_arm.goToPosition(
              new ArmPosition(
                  Arm.Positions.L4_PIV.mainPivotAngle(),
                  ElevatorConstants.MIN_PADDED_LENGTH,
                  WristConstants.K_G_ANGLE_WITH_CORAL)));
  public final Function<Integer, POI> selectedPOI;
  public final Function<Autos, Command> scoringPosition;
  public final double outtakeVoltage;
  public final ArmPosition arm;
  public final Function<Autos, Command> stow;
  public final Function<Autos, Command> premove;

  private ReefScoringOption(
      Function<Integer, POI> selectedPOI,
      Function<Autos, Command> scoringPosition,
      double outtakeVoltage,
      ArmPosition arm,
      Function<Autos, Command> stow,
      Function<Autos, Command> premove) {
    this.selectedPOI = selectedPOI;
    this.scoringPosition = scoringPosition;
    this.outtakeVoltage = outtakeVoltage;
    this.arm = arm;
    this.stow = stow;
    this.premove = premove;
  }
}
