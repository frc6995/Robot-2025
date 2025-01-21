// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AccelerationUnit;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaePivotS.AlgaePivotConstants;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;
import edu.wpi.first.wpilibj.simulation.TiltedElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

@Logged
public class ElevatorS extends SubsystemBase {
  public class ElevatorConstants {
    public static final int LEADER_ID = 40;
    public static final int FOLLOWER_ID = 41;
    public static final double MOTOR_ROTATIONS_PER_METER = 60;
    public static final Per<AngleUnit, DistanceUnit> MOTOR_ROTATIONS_PER_METER_UNIT = Rotations.per(Meter)
        .ofNative(MOTOR_ROTATIONS_PER_METER);
    public static final Distance MIN_LENGTH = Inches.of(27.0);
    public static final Distance MAX_LENGTH = Inches.of(67.0);

    private static TalonFXConfiguration configureLeader(TalonFXConfiguration config) {
      config.MotorOutput
          .withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.CounterClockwise_Positive);
      config.SoftwareLimitSwitch
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(MAX_LENGTH.in(Meters) * ElevatorConstants.MOTOR_ROTATIONS_PER_METER)
          .withReverseSoftLimitEnable(true)

          .withReverseSoftLimitThreshold(MIN_LENGTH.in(Meters) * ElevatorConstants.MOTOR_ROTATIONS_PER_METER);
      config.CurrentLimits
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(Amps.of(50));

      return config;
    }

    private static TalonFXConfiguration configureFollower(TalonFXConfiguration config) {
      config.MotorOutput
          .withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.CounterClockwise_Positive);

      config.CurrentLimits
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(Amps.of(50));

      return config;
    }

    public static final PerUnit<VoltageUnit, AngularVelocityUnit> VoltsPerRotationPerSecond = Volts
        .per(RotationsPerSecond);
    public static final PerUnit<VoltageUnit, AngularAccelerationUnit> VoltsPerRotationPerSecondSquared = Volts
        .per(RotationsPerSecond.per(Second));
    public static final Per<VoltageUnit, AngularVelocityUnit> K_V = VoltsPerRotationPerSecond.ofNative(12.0/100.0);
    public static final Per<VoltageUnit, AngularAccelerationUnit> K_A = VoltsPerRotationPerSecondSquared
        .ofNative(0.12185);

    public static final LinearSystem<N2, N1, N2> PLANT = LinearSystemId
        .identifyPositionSystem(
            K_V.in(VoltsPerRotationPerSecond) * MOTOR_ROTATIONS_PER_METER,
            K_A.in(VoltsPerRotationPerSecondSquared) * MOTOR_ROTATIONS_PER_METER);
    public static final double K_G = 0.1;

  }

  private TalonFX leader = new TalonFX(ElevatorConstants.LEADER_ID);
  private TalonFX follower = new TalonFX(ElevatorConstants.FOLLOWER_ID);
  private StatusSignal<Angle> positionSignal = leader.getPosition();
  private TiltedElevatorSim sim = new TiltedElevatorSim(
      ElevatorConstants.PLANT, DCMotor.getKrakenX60(2),
      ElevatorConstants.MOTOR_ROTATIONS_PER_METER,
      Units.inchesToMeters(1.5) * 2,
      ElevatorConstants.MIN_LENGTH.in(Meters),
      ElevatorConstants.MAX_LENGTH.in(Meters), false);

  public final MechanismLigament2d ELEVATOR = new MechanismLigament2d(
      "elevator", ElevatorConstants.MIN_LENGTH.in(Meters), 0, 4, new Color8Bit(235, 137, 52));    
  /** Creates a new ElevatorS. */

  public ElevatorS() {
    leader.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    var config = new TalonFXConfiguration();
    leader.getConfigurator().refresh(config);
    leader.getConfigurator().apply(
        ElevatorConstants.configureLeader(
            config));

    var followerConfig = new TalonFXConfiguration();
    follower.getConfigurator().refresh(followerConfig);
    follower.getConfigurator().apply(
        ElevatorConstants.configureFollower(
            followerConfig));

    follower.setControl(new Follower(ElevatorConstants.LEADER_ID, false));
    setDefaultCommand(this.stop());
  }

  public double getLengthMeters() {
    return getMotorRotations() / ElevatorConstants.MOTOR_ROTATIONS_PER_METER;
  }

  public double getMotorRotations() {
    return positionSignal.getValueAsDouble();
  }

  public double elevatorAngle = 0;

  public void setElevatorAngle(double angleRadians) {
    elevatorAngle = angleRadians;
  }

  private double getKGVolts() {
    return ElevatorConstants.K_G * Math.sin(elevatorAngle);
  }

  public void simulationPeriodic() {
    var simState = leader.getSimState();
    simState.setSupplyVoltage(12);
    // simState.getMotorVoltage is counterclockwise negative
    double volts = simState.getMotorVoltage();
    sim.setInput(volts - getKGVolts());
    sim.update(0.02);
    var rotorPos = sim.getPositionMeters()
        * ElevatorConstants.MOTOR_ROTATIONS_PER_METER;
    var rotorVel = sim.getVelocityMetersPerSecond()
        * ElevatorConstants.MOTOR_ROTATIONS_PER_METER;
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
    return this.run(()->leader.setControl(voltage.withOutput(1)));
  }
  public Command down() {
    return this.run(()->leader.setControl(voltage.withOutput(-1)));
  }
  public Command stop() {
    return this.run(()->leader.setControl(voltage.withOutput(0)));
  }
}
