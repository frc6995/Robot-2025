// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.elevator.RealElevatorS.ElevatorConstants;
import frc.robot.util.NomadMathUtil;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Logged
public class RealWristS extends Wrist {
  public class WristConstants {

    // [Things related to hardware] such as motor hard limits, can ids, pid constants, motor
    // rotations per arm rotation.

    public static final Angle CCW_LIMIT = Degrees.of(85);
    public static final Angle CW_LIMIT = Degrees.of(-127);
    public static final double MOTOR_ROTATIONS_PER_ARM_ROTATION = 70;
    // Units=volts/pivot rotation/s
    public static final double K_V = 9.2;
    public static final double K_A = 0.04;
    public static final double CG_DIST = Units.inchesToMeters(10);
    public static final LinearSystem<N2, N1, N2> PLANT =
        LinearSystemId.identifyPositionSystem(
            Units.radiansToRotations(K_V), Units.radiansToRotations(K_A));

    // Pose

    // CAN IDs
    public static final int LEADER_CAN_ID = 50;

    public static final int CURRENT_LIMIT = 100;

    public static final double OUT_VOLTAGE = 0;
    public static final double IN_VOLTAGE = 0;

    public static final double K_G = 0.5;
    public static final double K_S = 0;
    // arm plus hand
    public static final Mass ARM_MASS = Pounds.of(16).plus(Pounds.of(0));
    public static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
    public static final double MOI = 0.10829;
    public static TalonFXConfiguration configureLeader(TalonFXConfiguration config) {
      config.Slot0.withKS(K_S).withKV(K_V).withKA(K_A).withKP(10).withKD(1);
      config.MotionMagic.withMotionMagicCruiseVelocity(2).withMotionMagicAcceleration(2);
      config.Feedback
          // .withFeedbackRemoteSensorID(34)
          // .withFeedbackSensorSource(FeedbackSensorSourceValue.SyncCANcoder)
          .withSensorToMechanismRatio(MOTOR_ROTATIONS_PER_ARM_ROTATION);
      config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(CCW_LIMIT)
          .withReverseSoftLimitThreshold(CW_LIMIT)
          .withReverseSoftLimitEnable(true);
      return config;
    }

  }

  private final SingleJointedArmSim m_pivotSim =
      new SingleJointedArmSim(
        WristConstants.GEARBOX, WristConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION, 
        WristConstants.MOI, 0.2, WristConstants.CW_LIMIT.in(Radians), 
        WristConstants.CCW_LIMIT.in(Radians), false, 0
      );

  public final MechanismLigament2d WRIST =
      new MechanismLigament2d("wrist", 0, 0, 4, new Color8Bit(235, 137, 52));
  private TalonFX m_leader = new TalonFX(WristConstants.LEADER_CAN_ID);
  private MotionMagicVoltage m_profileReq = new MotionMagicVoltage(0);
  private VoltageOut m_voltageReq = new VoltageOut(0);
  private StatusSignal<Angle> m_angleSig = m_leader.getPosition();
  private double m_setpointRotations;

  private DoubleSupplier m_mainAngleSupplier = () -> 0;
  public void setMainAngleSupplier(DoubleSupplier mainAngleSupplier) {
    m_mainAngleSupplier = mainAngleSupplier;
  }
  /** Creates a new MainPivotS. */
  public RealWristS() {
    m_leader
        .getConfigurator()
        .apply(WristConstants.configureLeader(new TalonFXConfiguration()));
    m_leader.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    m_pivotSim.setState(VecBuilder.fill(WristConstants.CW_LIMIT.in(Radians), 0));
    setDefaultCommand(hold());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    WRIST.setAngle(Units.rotationsToDegrees(m_angleSig.getValueAsDouble()));
  }

  public void simulationPeriodic() {
    for (int i = 0; i < 2; i++) {
      var simState = m_leader.getSimState();
      simState.setSupplyVoltage(12);
      // simState.getMotorVoltage is counterclockwise negative
      double volts = simState.getMotorVoltage();

      m_pivotSim.setInput(
          NomadMathUtil.subtractkS(volts, WristConstants.K_S)
          // - getKgVolts()
          );
      m_pivotSim.update(0.01);
      var rotorPos =
          m_pivotSim.getAngleRads()
              * WristConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION
              / (2 * Math.PI);
      var rotorVel =
          m_pivotSim.getVelocityRadPerSec()
              * WristConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION
              / (2 * Math.PI);

      simState.setRawRotorPosition(rotorPos);
      simState.setRotorVelocity(rotorVel);
    }
  }

  public double getAngleRotations() {
    return m_angleSig.getValueAsDouble();
  }

  public double getAngleRadians() {
    return Units.rotationsToRadians(m_angleSig.getValueAsDouble());
  }

  public double getKgVolts() {
    return Math.cos(getAngleRadians() + m_mainAngleSupplier.getAsDouble())*WristConstants.K_G;
  }

  public void setAngleRadians(double angle) {
    m_setpointRotations = Units.radiansToRotations(angle);

    m_leader.setControl(
        m_profileReq.withPosition(m_setpointRotations)); // .withFeedForward(getKgVolts()));
  }

  public Command goTo(DoubleSupplier angleSupplier) {
    return run(() -> setAngleRadians(angleSupplier.getAsDouble()));
  }

  public Command goTo(Supplier<Angle> angleSupplier) {
    return run(() -> setAngleRadians(angleSupplier.get().in(Radians)));
  }

  public Command voltage(DoubleSupplier voltageSupplier) {
    return run(
        () -> {
          m_leader.setControl(m_voltageReq.withOutput(voltageSupplier.getAsDouble()));
        });
  }

  public Command hold() {
    return sequence(runOnce(() -> setAngleRadians(getAngleRadians())), Commands.idle());
  }
}
