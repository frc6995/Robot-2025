package frc.robot.subsystems.arm.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public abstract class Elevator extends SubsystemBase {
  public abstract Command goToLength(DoubleSupplier lengthSupplier);
}
