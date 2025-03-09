package frc.robot.util.setpointGenerator3173;

public class Types {
    public record Constraints(double maxVelocity, double maxAcceleration) {
  }

  public record ChassisConstraints(Constraints translation, Constraints rotation){}
}
