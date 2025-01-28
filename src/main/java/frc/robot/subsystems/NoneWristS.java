package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;

@Logged
public class NoneWristS extends Wrist {

    @Override
    public double getAngle() {
       return position;
    }

    @Override
    public Command goTo(DoubleSupplier angleSupplier) {
        // TODO Auto-generated method stub
        return run(()->position = angleSupplier.getAsDouble());
    }
    private double position = 0;
}
