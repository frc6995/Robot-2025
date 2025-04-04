package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Hand extends SubsystemBase{
  public Hand() {
    setDefaultCommand(stop());
  }
  public abstract Command stop();
  public abstract Command inCoral();
  public abstract Command outCoral();
  public abstract Command inAlgae();
  public abstract Command outAlgae();
  public abstract Command outAlgaeSlow();
}
