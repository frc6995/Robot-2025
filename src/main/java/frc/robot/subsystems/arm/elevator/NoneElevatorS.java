package frc.robot.subsystems.arm.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public abstract class NoneElevatorS extends Elevator {
  public abstract Command goToLength(DoubleSupplier lengthSupplier);
}
