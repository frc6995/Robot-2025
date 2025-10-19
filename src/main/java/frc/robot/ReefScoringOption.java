package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Function;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.IntakeS.HandConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.elevator.RealElevatorS.ElevatorConstants;
import frc.robot.subsystems.arm.wrist.RealWristS.WristConstants;



public enum ReefScoringOption{


    L1(
      POI::selectedL1POI, (autos)->autos.m_arm.goToPosition(Arm.Positions.L1),
      -1,//ignored by bindings
      Arm.Positions.L1, 0,
      (autos)->autos.m_arm.goToPosition(Arm.Positions.L1),
      (autos)->new ScheduleCommand(autos.m_arm.goToPosition(Arm.Positions.L1)) // so tapping the align button goes to L1
      ),
      
    L2(
      POI::selectedBatterySidePOI, (autos)->autos.m_arm.goToPosition(Arm.Positions.L2_OPP),
      -4, Arm.Positions.L2_OPP, 0,
      (autos)->autos.m_arm.goToPosition(
        // intentional, we want a pivot up
        new ArmPosition(Arm.Positions.L3_OPP.mainPivotAngle(), ElevatorConstants.MIN_PADDED_LENGTH, Arm.Positions.L2_OPP.wristAngle())),
      
      (autos)->new ScheduleCommand(autos.m_arm.goToPosition(Arm.Positions.L2_OPP)) // so tapping the align button goes to L1
      ),
    L3(
      POI::selectedBatterySidePOI, (autos)->autos.m_arm.goToPosition(Arm.Positions.L3_OPP),
      -4, Arm.Positions.L3_OPP, 0,
      (autos)->autos.m_arm.goToPosition(
        new ArmPosition(Arm.Positions.L3_OPP.mainPivotAngle(), ElevatorConstants.MIN_PADDED_LENGTH, Arm.Positions.L3_OPP.wristAngle())),
      (autos)->autos.m_arm.goToPosition(
        new ArmPosition(Arm.Positions.L3_OPP.mainPivotAngle(), ElevatorConstants.MIN_PADDED_LENGTH, Arm.Positions.L3_OPP.wristAngle()))
    ),
    L3_HIGH_ALG(
      POI::selectedBatterySidePOI, (autos)->autos.m_arm.goToPosition(Arm.Positions.L3_HIGH_ALG),
      HandConstants.OUT_CORAL_VOLTAGE, Arm.Positions.L3_HIGH_ALG, 0,
      (autos)->autos.m_arm.goToPosition(
        new ArmPosition(Arm.Positions.L3_HIGH_ALG.mainPivotAngle(), ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(0))),
      (autos)->autos.m_arm.goToPosition(
        new ArmPosition(Arm.Positions.L3_HIGH_ALG.mainPivotAngle(), ElevatorConstants.MIN_PADDED_LENGTH, Radians.of(0)))
    ),
    L4(
      POI::selectedBatterySidePOI, (autos)->autos.m_arm.goToPosition(Arm.Positions.L4_OPP),
      //HandConstants.OUT_CORAL_VOLTAGE_REVERSE
      -4, Arm.Positions.L4_OPP, -Units.inchesToMeters(2),
      (autos)->autos.m_arm.goToPosition(
        new ArmPosition(Arm.Positions.L4_OPP.mainPivotAngle(), ElevatorConstants.MIN_PADDED_LENGTH, Arm.Positions.L4_OPP.wristAngle())),
      (autos)->autos.m_arm.goToPosition(
        new ArmPosition(Arm.Positions.L4_OPP.mainPivotAngle(), ElevatorConstants.MIN_PADDED_LENGTH, Arm.Positions.L4_OPP.wristAngle()))
    ),
    L3_PIV(
      POI::selectedPivotSidePOI, (autos)->autos.m_arm.goToPosition(Arm.Positions.L3),
      4, Arm.Positions.L3, 0,
      (autos)->autos.m_arm.goToPosition(
        new ArmPosition(Arm.Positions.L3_OPP.mainPivotAngle(), ElevatorConstants.MIN_PADDED_LENGTH, Arm.Positions.L3.wristAngle())),
      (autos)->autos.m_arm.goToPosition(
        new ArmPosition(Arm.Positions.L3.mainPivotAngle(), ElevatorConstants.MIN_PADDED_LENGTH, Arm.Positions.L3.wristAngle()))
    ),
    L4_PIV(
      POI::selectedPivotSidePOI, (autos)->autos.m_arm.goToPosition(Arm.Positions.L4),
      4, Arm.Positions.L4, 0,
      (autos)->Commands.sequence(
        autos.m_arm.goToPosition(
          new ArmPosition(
            Arm.Positions.L4.mainPivotAngle().minus(Degrees.of(5)),
            Arm.Positions.L4.elevatorLength(),
            Arm.Positions.L3.wristAngle()))

                      .until(()->
                        autos.m_arm.position.pivotRadians() < Arm.Positions.L4.pivotRadians()-Units.degreesToRadians(4)
                      )
                      .withTimeout(1),
                  autos.m_arm.goToPosition(
                    new ArmPosition(Arm.Positions.L3_OPP.mainPivotAngle(), ElevatorConstants.MIN_PADDED_LENGTH, Arm.Positions.L3_OPP.wristAngle()))),
                    
                    (autos)->autos.m_arm.goToPosition(
                      new ArmPosition(Arm.Positions.L4.mainPivotAngle(), ElevatorConstants.MIN_PADDED_LENGTH, WristConstants.K_G_ANGLE_WITH_CORAL))
                )
                ;
                public final Function<Integer,POI> selectedPOI;
                public final Function<Autos,Command> scoringPosition;
                public final double outtakeVoltage;
                public final ArmPosition arm;

    public final double coralOffsetMeters;
    public final Function<Autos,Command> stow;
    public final Function<Autos,Command> premove;
    private ReefScoringOption(Function<Integer,POI> selectedPOI, Function<Autos,Command> scoringPosition, double outtakeVoltage, ArmPosition arm, double coralOffsetMeters, Function<Autos,Command> stow,
    Function<Autos,Command> premove){
      this.selectedPOI = selectedPOI;
      this.scoringPosition = scoringPosition;
      this.outtakeVoltage = outtakeVoltage;
      this.arm = arm;
      this.coralOffsetMeters = coralOffsetMeters;
      this.stow = stow;
      this.premove = premove;
    }
  }