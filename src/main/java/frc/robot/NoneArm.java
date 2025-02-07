package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorS.ElevatorConstants;
import frc.robot.subsystems.MainPivotS.MainPivotConstants;
import frc.robot.subsystems.WristS.WristConstants;

public class NoneArm extends Arm {

  public NoneArm() {
    ARM = new MechanismLigament2d("arm-pivot", 0, 0);
    this.position = new ArmPosition(
      MainPivotConstants.CW_LIMIT,
      ElevatorConstants.MIN_LENGTH,
      WristConstants.CW_LIMIT
    );
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
}
