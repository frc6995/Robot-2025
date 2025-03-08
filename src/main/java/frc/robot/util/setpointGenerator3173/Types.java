package frc.robot.util.setpointGenerator3173;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class Types {
    public record Constraints(double maxVelocity, double maxAcceleration) {
  }

  public record ChassisConstraints(Constraints translation, Constraints rotation){}
}
