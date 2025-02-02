package frc.robot.subsystems.arm.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Elevator extends SubsystemBase {
    public abstract Command goToLength(DoubleSupplier lengthSupplier);
}
