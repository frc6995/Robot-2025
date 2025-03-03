package frc.robot.subsystems.arm.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class NoneElevatorS extends Elevator {
    public abstract Command goToLength(DoubleSupplier lengthSupplier);
}
