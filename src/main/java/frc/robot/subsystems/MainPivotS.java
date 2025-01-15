// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;

public class MainPivotS extends SubsystemBase {
  public class MainPivotConstants {
    
  }

  public final MechanismLigament2d MAIN_PIVOT = new MechanismLigament2d(
    "main_pivot", 8, 0, 4, new Color8Bit(235, 137, 52));
  /** Creates a new MainPivotS. */
  public MainPivotS() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simulationPeriodic() {

  }

  public Command goTo(double armRotations) {
    return none();
  }

  public Command hold() {
    return none();
  }


}
