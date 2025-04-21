package frc.robot.subsystems.arm.wrist;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

@Logged
public class NoneWristS extends Wrist {

  @Override
  public double getAngleRadians() {
    return position;
  }

  @Override
  public Command goTo(DoubleSupplier angleSupplierRadians) {
    // TODO Auto-generated method stub
    return run(() -> position = angleSupplierRadians.getAsDouble());
  }

  private double position = 0;
}
