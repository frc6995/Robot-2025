package frc.robot.subsystems.arm.wrist;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.Commands.run;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Wrist extends SubsystemBase {
    public abstract double getAngleRadians();
    public abstract Command goTo(DoubleSupplier angleSupplier);

  public Command goTo(Supplier<Angle> angleSupplier) {
    return goTo(()->angleSupplier.get().in(Radians));
  }

}
