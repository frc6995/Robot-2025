// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class AlgaePivotS extends SubsystemBase {
  public class AlgaePivotConstants {
    public static final int MOTOR_ID = 50;
    public static final Angle DEPLOY_ANGLE = Degrees.of(10);
    public static final Angle RETRACT_ANGLE = Degrees.of(90);
    public static final Per<VoltageUnit, AngularVelocityUnit> K_V =
        VoltsPerRadianPerSecond.ofNative(1.3684);
    public static final Per<VoltageUnit, AngularAccelerationUnit> K_A =
        VoltsPerRadianPerSecondSquared.ofNative(0.012185);

    public static final LinearSystem<N2, N1, N2> PLANT =
        LinearSystemId.identifyPositionSystem(
            K_V.in(VoltsPerRadianPerSecond), K_A.in(VoltsPerRadianPerSecondSquared));
    public static final double MOTOR_ROTATIONS_PER_PIVOT_ROTATION = 60.0;

    private static TalonFXConfiguration configureMotor(TalonFXConfiguration config) {
      config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.CounterClockwise_Positive);
      config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(RETRACT_ANGLE)
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(DEPLOY_ANGLE);
      config.CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(Amps.of(50));
      config.Feedback.withSensorToMechanismRatio(MOTOR_ROTATIONS_PER_PIVOT_ROTATION);

      return config;
    }
  }

  private final SingleJointedArmSim m_pivotSim =
      new SingleJointedArmSim(
          AlgaePivotConstants.PLANT,
          DCMotor.getKrakenX60(1),
          AlgaePivotConstants.MOTOR_ROTATIONS_PER_PIVOT_ROTATION,
          Units.inchesToMeters(10),
          AlgaePivotConstants.DEPLOY_ANGLE.in(Radians),
          AlgaePivotConstants.RETRACT_ANGLE.in(Radians),
          false,
          AlgaePivotConstants.RETRACT_ANGLE.in(Radians),
          new double[0]);

  TalonFX motor = new TalonFX(AlgaePivotConstants.MOTOR_ID);

  public final MechanismLigament2d ALGAE_PIVOT =
      new MechanismLigament2d(
          "algae-pivot-1", Units.inchesToMeters(5), 0, 4, new Color8Bit(235, 137, 52));
  private final MechanismLigament2d ALGAE_PIVOT_2 =
      new MechanismLigament2d(
          "algae-pivot-2", Units.inchesToMeters(5), 60, 4, new Color8Bit(235, 137, 52));
  private final MechanismLigament2d ALGAE_PIVOT_3 =
      new MechanismLigament2d(
          "algae-pivot-3", Units.inchesToMeters(5), -60, 4, new Color8Bit(235, 137, 52));
  private final MechanismLigament2d ALGAE_PIVOT_4 =
      new MechanismLigament2d(
          "algae-pivot-4", Units.inchesToMeters(5), -60, 4, new Color8Bit(235, 137, 52));

  private void setupVisualizer() {
    ALGAE_PIVOT.append(ALGAE_PIVOT_2).append(ALGAE_PIVOT_3).append(ALGAE_PIVOT_4);
  }

  public final VoltageOut voltageReq = new VoltageOut(0);
  public final StatusSignal<Angle> position = motor.getPosition();

  /** Creates a new AlgaePivotS. */
  public AlgaePivotS() {
    setupVisualizer();
    var config = new TalonFXConfiguration();
    motor.getConfigurator().refresh(config);
    motor.getConfigurator().apply(AlgaePivotConstants.configureMotor(config));
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(position);
    ALGAE_PIVOT.setAngle(Units.rotationsToDegrees(position.getValueAsDouble()));
  }

  public void simulationPeriodic() {
    var simState = motor.getSimState();
    simState.setSupplyVoltage(12);
    // simState.getMotorVoltage is counterclockwise negative
    double volts = simState.getMotorVoltage();
    m_pivotSim.setInput(volts);
    m_pivotSim.update(0.02);
    var rotorPos =
        Units.radiansToRotations(m_pivotSim.getAngleRads())
            * AlgaePivotConstants.MOTOR_ROTATIONS_PER_PIVOT_ROTATION;
    var rotorVel =
        Units.radiansToRotations(m_pivotSim.getVelocityRadPerSec())
            * AlgaePivotConstants.MOTOR_ROTATIONS_PER_PIVOT_ROTATION;
    simState.setRawRotorPosition(rotorPos);
    simState.setRotorVelocity(rotorVel);
  }

  public Command retract() {
    return this.run(() -> motor.setControl(voltageReq.withOutput(10)));
  }

  public Command deploy() {
    return this.run(() -> motor.setControl(voltageReq.withOutput(-10)));
  }
}
