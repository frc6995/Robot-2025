// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.simulation.TiltedElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.pivot.MainPivotS.MainPivotConstants;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Logged
public class RealElevatorS extends Elevator {
  public class ElevatorConstants {
    public static final int LEADER_ID = 40;
    public static final int FOLLOWER_ID = 41;
    // Approved by CAD
    public static final Per<AngleUnit, DistanceUnit> MOTOR_ROTATIONS_PER_METER_UNIT =
        Rotations.of(1).div(Inches.of(1.32278 * 2));
    public static final double MOTOR_ROTATIONS_PER_METER =
        MOTOR_ROTATIONS_PER_METER_UNIT.in(Rotations.per(Meter));

    public static final Distance MIN_LENGTH = Inches.of(27.0);
    public static final Distance MAX_LENGTH = Inches.of(67.0);
    public static final double MIN_LENGTH_ROTATIONS =
        MIN_LENGTH.in(Meters) * ElevatorConstants.MOTOR_ROTATIONS_PER_METER;
    public static final double MAX_LENGTH_ROTATIONS =
        MAX_LENGTH.in(Meters) * ElevatorConstants.MOTOR_ROTATIONS_PER_METER;

    private static TalonFXConfiguration configureLeader(TalonFXConfiguration config) {
      config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.CounterClockwise_Positive);
      config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(MAX_LENGTH_ROTATIONS)
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(MIN_LENGTH_ROTATIONS);
      config.CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(50));
      config.Slot0.withKS(0)
          .withKV(K_V.in(VoltsPerRotationPerSecond))
          .withKA(K_A.in(VoltsPerRotationPerSecondSquared))
          .withKP(1)
          .withKD(0);
      config.MotionMagic.withMotionMagicAcceleration(MOTOR_ROTATIONS_PER_METER * 7)
          .withMotionMagicCruiseVelocity(MOTOR_ROTATIONS_PER_METER * 4);
      return config;
    }

    private static TalonFXConfiguration configureFollower(TalonFXConfiguration config) {
      config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.CounterClockwise_Positive);

      config.CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(50));

      return config;
    }

    public static final PerUnit<VoltageUnit, AngularVelocityUnit> VoltsPerRotationPerSecond =
        Volts.per(RotationsPerSecond);
    public static final PerUnit<VoltageUnit, AngularAccelerationUnit>
        VoltsPerRotationPerSecondSquared = Volts.per(RotationsPerSecond.per(Second));
    public static final Per<VoltageUnit, AngularVelocityUnit> K_V =
        VoltsPerRotationPerSecond.ofNative(12.0 / 100.0);

    public static final Per<VoltageUnit, AngularAccelerationUnit> K_A =
        VoltsPerRotationPerSecondSquared.ofNative(0.06 / MOTOR_ROTATIONS_PER_METER);

    public static final LinearSystem<N2, N1, N2> PLANT =
        LinearSystemId.identifyPositionSystem(
            K_V.in(VoltsPerRotationPerSecond) * MOTOR_ROTATIONS_PER_METER,
            K_A.in(VoltsPerRotationPerSecondSquared) * MOTOR_ROTATIONS_PER_METER);
    public static final double K_G = 0.1;
    public static final InterpolatingDoubleTreeMap LENGTH_TO_MOI =
        InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(ElevatorConstants.MIN_LENGTH.in(Meters), 0.7419),
            Map.entry(ElevatorConstants.MAX_LENGTH.in(Meters), 2.812027));

    public static double getMoI(double armLengthMeters) {
      return LENGTH_TO_MOI.get(armLengthMeters);
    }
  }

  private TalonFX leader = new TalonFX(ElevatorConstants.LEADER_ID);
  private TalonFX follower = new TalonFX(ElevatorConstants.FOLLOWER_ID);
  private PositionVoltage positionReq = new PositionVoltage(ElevatorConstants.MIN_LENGTH_ROTATIONS);
  private MotionMagicVoltage profileReq =
      new MotionMagicVoltage(ElevatorConstants.MIN_LENGTH_ROTATIONS);
  private StatusSignal<Angle> positionSignal = leader.getPosition();
  private TiltedElevatorSim sim =
      new TiltedElevatorSim(
          ElevatorConstants.PLANT,
          DCMotor.getKrakenX60(2),
          ElevatorConstants.MOTOR_ROTATIONS_PER_METER,
          Units.inchesToMeters(1.5) * 2,
          ElevatorConstants.MIN_LENGTH.in(Meters),
          ElevatorConstants.MAX_LENGTH.in(Meters),
          false);

  public final MechanismLigament2d ELEVATOR =
      new MechanismLigament2d(
          "elevator", ElevatorConstants.MIN_LENGTH.in(Meters), 0, 4, new Color8Bit(235, 137, 52));

  private DoubleSupplier angleRadiansSupplier = () -> MainPivotConstants.CW_LIMIT.in(Radians);

  /** Creates a new ElevatorS. */
  public RealElevatorS() {
    leader.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    var config = new TalonFXConfiguration();
    leader.getConfigurator().refresh(config);
    leader.getConfigurator().apply(ElevatorConstants.configureLeader(config));

    var followerConfig = new TalonFXConfiguration();
    follower.getConfigurator().refresh(followerConfig);
    follower.getConfigurator().apply(ElevatorConstants.configureFollower(followerConfig));

    follower.setControl(new Follower(ElevatorConstants.LEADER_ID, false));
    setDefaultCommand(this.hold());
  }

  public double getMoI() {
    return ElevatorConstants.getMoI(getLengthMeters());
  }
  public Command hold() {
    return this.runOnce(() -> positionReq.withPosition(getMotorRotations()).withVelocity(0))
        .andThen(this.run(() -> leader.setControl(positionReq)));
  }

  public double getLengthMeters() {
    return getMotorRotations() / ElevatorConstants.MOTOR_ROTATIONS_PER_METER;
  }

  public double getMotorRotations() {
    return positionSignal.getValueAsDouble();
  }

  public double getKGVolts() {
    return ElevatorConstants.K_G * Math.sin(angleRadiansSupplier.getAsDouble());
  }

  public void simulationPeriodic() {
    var simState = leader.getSimState();
    simState.setSupplyVoltage(12);
    // simState.getMotorVoltage is counterclockwise negative
    double volts = simState.getMotorVoltage();
    sim.setInput(volts - getKGVolts());
    sim.update(0.02);
    var rotorPos = sim.getPositionMeters() * ElevatorConstants.MOTOR_ROTATIONS_PER_METER;
    var rotorVel = sim.getVelocityMetersPerSecond() * ElevatorConstants.MOTOR_ROTATIONS_PER_METER;
    simState.setRawRotorPosition(rotorPos);
    simState.setRotorVelocity(rotorVel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    BaseStatusSignal.refreshAll(positionSignal);
    ELEVATOR.setLength(getLengthMeters());
  }

  VoltageOut voltage = new VoltageOut(0);

  public Command up() {
    return this.run(() -> leader.setControl(voltage.withOutput(1)));
  }

  public Command down() {
    return this.run(() -> leader.setControl(voltage.withOutput(-1)));
  }

  public Command stop() {
    return this.run(() -> leader.setControl(voltage.withOutput(0)));
  }

  private void goToRotations(double motorRotations) {
    leader.setControl(profileReq.withPosition(motorRotations).withFeedForward(getKGVolts()));
  }

  public Command goToLength(DoubleSupplier length) {
    return this.run(
        () -> goToRotations(length.getAsDouble() * ElevatorConstants.MOTOR_ROTATIONS_PER_METER));
  }

  public Command goToLength(Supplier<Distance> length) {
    return this.run(
        () -> goToRotations(length.get().in(Meters) * ElevatorConstants.MOTOR_ROTATIONS_PER_METER));
  }

  public void setAngleSupplier(DoubleSupplier angleRadians) {
    angleRadiansSupplier = angleRadians;
  }
}
