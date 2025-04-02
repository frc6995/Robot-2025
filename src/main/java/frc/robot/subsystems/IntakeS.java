// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CoralSensor;
import frc.robot.util.Capture;

@Logged
public class IntakeS extends SubsystemBase {
  public final MechanismLigament2d TOP_ROLLER = new MechanismLigament2d(
      "top-roller", 0.05, 0, 4, new Color8Bit(Color.kWhite));
  public final MechanismLigament2d BOTTOM_ROLLER = new MechanismLigament2d(
      "top-roller", 0.05, 0, 4, new Color8Bit(Color.kWhite));

  public class HandConstants {

    public static final double CORAL_LENGTH_METERS = Units.inchesToMeters(11.875);
    public static final int CAN_ID = 51;

    public static final double IN_CORAL_VOLTAGE = 4;

    public static final double OUT_CORAL_VOLTAGE = -6; // worked with -6 but coral bounced
    public static final double OUT_CORAL_VOLTAGE_SLOW = -4; // worked with -6 but coral bounced
    public static final double OUT_CORAL_VOLTAGE_REVERSE = 6;
    public static final double IN_ALGAE_VOLTAGE = -10;

    public static final double OUT_ALGAE_VOLTAGE = 4;
    public static final double OUT_ALGAE_VOLTAGE_SLOW = 2;

    public static final double CLEAR_SENSOR_OFFSET = -Units.inchesToMeters(5.2);
    public static final double GROUND_INTAKE_OFFSET = -Units.inchesToMeters(0);
    public static final double CORAL_METERS_PER_WHEEL_ROT = (2 * Math.PI* Units.inchesToMeters(3));

    public static TalonFXConfiguration configureMotor(TalonFXConfiguration config) {
      config.CurrentLimits.withStatorCurrentLimit(90).withStatorCurrentLimitEnable(true);
      config.Feedback.SensorToMechanismRatio = 16.0 / 3.0;
      // volts per wheel rotation = 8-9 inches of coral
      config.Slot0.withKP(6);
      return config;
    }
  }

  private final TalonFX motor = new TalonFX(HandConstants.CAN_ID);

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withPosition(0);
  private final DCMotorSim motorSim = new DCMotorSim(
      LinearSystemId.identifyPositionSystem(Units.radiansToRotations(0.12), Units.radiansToRotations(0.003)),
      DCMotor.getKrakenX60(1));

  public void simulationPeriodic() {
    var simState = motor.getSimState();
    simState.setSupplyVoltage(12);
    // simState.getMotorVoltage is counterclockwise negative
    double volts = simState.getMotorVoltage();
    motorSim.setInput(volts);
    motorSim.update(0.02);
    var rotorPos = motorSim.getAngularPositionRotations();
    var rotorVel = motorSim.getAngularVelocityRPM() / 60.0;
    simState.setRawRotorPosition(rotorPos);
    simState.setRotorVelocity(rotorVel);
  }

  public CoralSensor m_coralSensor = new CoralSensor();

  public StatusSignal<Angle> m_positionSig = motor.getPosition();
  public StatusSignal<Voltage> m_voltageSig = motor.getMotorVoltage();
  private Optional<Double> lastRotationsAtSensorTrip = Optional.empty();

  public double lastRotationsAtSensorTrip() {
    return lastRotationsAtSensorTrip.orElse(0.0);
  }

  // 0 is with the coral centered on the sensor
  private Optional<Double> coralPositionAtSensorTrip = Optional.empty();

  /** Creates a new HandRollerS. */
  public IntakeS() {
    super();
    motor.getConfigurator().apply(HandConstants.configureMotor(new TalonFXConfiguration()));
    setDefaultCommand(stop());
    new Trigger(m_coralSensor::hasCoral).onTrue(
        Commands.runOnce(this::setHasCoral).ignoringDisable(true));
    if (RobotBase.isSimulation()) {
      new Trigger(() -> {
        m_voltageSig.refresh();
        return getCoralInlineOffset() * Math.signum(m_voltageSig.getValueAsDouble()) > Units.inchesToMeters(6);
      }).onTrue(
          Commands.runOnce(this::setHasNoCoral).ignoringDisable(true));
    }
  }

  private void setHasCoral() {
    m_positionSig.refresh();
    lastRotationsAtSensorTrip = Optional.of(m_positionSig.getValueAsDouble());
    if (m_voltageSig.getValueAsDouble() >= 0) {
      coralPositionAtSensorTrip = Optional.of(-HandConstants.CORAL_LENGTH_METERS / 2.0);
    } else {
      coralPositionAtSensorTrip = Optional.of(HandConstants.CORAL_LENGTH_METERS / 2.0);
    }

  }

  private void setHasNoCoral() {
    m_coralSensor.setHasCoral(false);
  }

  @Override
  public void periodic() {
    // TOP_ROLLER.setAngle(TOP_ROLLER.getAngle() + 4 * voltageRequest.Output);
    // BOTTOM_ROLLER.setAngle(BOTTOM_ROLLER.getAngle() - 4 * voltageRequest.Output);
    // This method will be called once per scheduler run
  }

  public Command driveToOffset(double offsetMeters) {
    Capture<Double> targetMotorRotations = new Capture<Double>(0.0);
    
    return either(
        runOnce(() -> {
          targetMotorRotations.inner = m_positionSig.getValueAsDouble()
              + ((offsetMeters - getCoralInlineOffset()) / HandConstants.CORAL_METERS_PER_WHEEL_ROT);
        }).andThen(run(() -> motor.setControl(positionRequest.withPosition(targetMotorRotations.inner)))
        
        ),
        voltage(0),
        () -> !lastRotationsAtSensorTrip.isEmpty()).onlyWhile(m_coralSensor::hasCoral).andThen(
          waitUntil(m_coralSensor::hasCoral)
        ).repeatedly();
  }

  public Command voltage(DoubleSupplier voltage) {
    return this.run(() -> motor.setControl(voltageRequest.withOutput(voltage.getAsDouble())));
  }

  public Command voltage(double voltage) {
    return voltage(() -> voltage);
  }

  public Command stop() {
    return voltage(0);
  }

  public Command inCoral() {
    return voltage(HandConstants.IN_CORAL_VOLTAGE);
  }

  public Command outCoral() {
    return voltage(HandConstants.OUT_CORAL_VOLTAGE);

  }

  public Command outCoralReverse() {
    return voltage(HandConstants.OUT_CORAL_VOLTAGE_REVERSE);

  }

  public Command outCoralSlow() {
    return voltage(HandConstants.OUT_CORAL_VOLTAGE_SLOW);

  }

  public Command inAlgae() {
    return voltage(HandConstants.IN_ALGAE_VOLTAGE);
  }

  public Command outAlgae() {
    return voltage(HandConstants.OUT_ALGAE_VOLTAGE);
  }

  public Command outAlgaeSlow() {
    return voltage(HandConstants.OUT_ALGAE_VOLTAGE_SLOW);
  }

  // positive meters means less intaken

  public double getCoralInlineOffset() {
    if (lastRotationsAtSensorTrip.isEmpty() || coralPositionAtSensorTrip.isEmpty()) {
      return 0;
    } else {
      m_positionSig.refresh();
      var deltaWheelPositionRotations = m_positionSig.getValueAsDouble() - lastRotationsAtSensorTrip.get();
      var deltaCoralPositionMeters = deltaWheelPositionRotations *HandConstants.CORAL_METERS_PER_WHEEL_ROT;
      return coralPositionAtSensorTrip.get() + deltaCoralPositionMeters;
    }
  }

}
