package frc.robot.subsystems.arm;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.elevator.RealElevatorS.ElevatorConstants;
import frc.robot.subsystems.arm.pivot.MainPivotS.MainPivotConstants;
import frc.robot.subsystems.arm.wrist.RealWristS.WristConstants;

public class NoneArm extends Arm {

  public NoneArm() {
    ARM = new MechanismLigament2d("arm-pivot", 0, 0);
    this.position =
        new ArmPosition(
            MainPivotConstants.CW_LIMIT, ElevatorConstants.MIN_LENGTH, WristConstants.CW_LIMIT);
  }

  @Override
  public MechanismLigament2d getMechanism() {
    return ARM;
  }

  public Command goToPosition(ArmPosition position) {
    return runOnce(() -> this.position = position);
  }

  @Override
  public void update() {}

  @Override
  public Command Climb() {
    // TODO Auto-generated method stub
    return Commands.none();
  }

  @Override
  public boolean readyToClimb() {
    // TODO Auto-generated method stub
    return false;
  }
}
