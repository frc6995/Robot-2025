// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

/* Changed the imports to be compatible */
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.NomadMathUtil;
import frc.robot.Robot;


public class MainPivotS extends SubsystemBase {
  public class MainPivotConstants {

    //[Things related to hardware] such as motor hard limits, can ids, pid constants, motor rotations per arm rotation.

    public static final double CCW_LIMIT = Units.degreesToRadians(100);
    public static final double CW_LIMIT = Units.degreesToRadians(40);

    public static final double K_V = 0;
    public static final double K_A = 0;
    public static final double CG_DIST = Units.inchesToMeters(6);
    public static final LinearSystem<N2, N1, N2> PLANT = LinearSystemId
        .identifyPositionSystem(Units.radiansToRotations(K_V), Units.radiansToRotations(K_A));

    //Pose
    public static final double CORALLVL1IN = Units.degreesToRadians(0);
    public static final double CORALLVL2IN = Units.degreesToRadians(0);
    public static final double CORALLVL3IN = Units.degreesToRadians(0);
    public static final double CORALLVL4IN = Units.degreesToRadians(0);
    public static final double CORALSTATIONIN1 = Units.degreesToRadians(0);
    public static final double CORALSTATIONIN2 = Units.degreesToRadians(0);
    public static final double ALGAELVL1IN = Units.degreesToRadians(0);
    public static final double ALGAELVL2IN = Units.degreesToRadians(0);
    public static final double ALGAEOUT = Units.degreesToRadians(0);
    public static final double SCORE_ANGLE = 2* Math.PI / 3.0 - Units.degreesToRadians(5);
    public static final double HANDOFF_ANGLE = MainPivotS.MainPivotConstants.CCW_LIMIT;

    //CAN IDs
    public static final int LEADER_CAN_ID = 0;
    public static final int FOLLOWER_CAN_ID = 0;
    public static final int OPPOSING1_CAN_ID = 0;
    public static final int OPPOSING2_CAN_ID = 0;

    public static final int CURRENT_LIMIT = 100;
   
    public static final double OUT_VOLTAGE = 0;
    public static final double IN_VOLTAGE = 0;

    public static final double MOTOR_ROTATIONS_PER_ARM_ROTATION = 70;

    public static final double K_G = 0;
    public static final double K_S = 0;
  }

  private final SingleJointedArmSim m_pivotSim = new SingleJointedArmSim(
      MainPivotConstants.PLANT,
      DCMotor.getKrakenX60(1),
      MainPivotConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION,
      MainPivotConstants.CG_DIST,
      MainPivotConstants.CW_LIMIT,
      MainPivotConstants.CCW_LIMIT,
      false,
      MainPivotConstants.CW_LIMIT);


  public final MechanismLigament2d MAIN_PIVOT = new MechanismLigament2d(
    "main_pivot", 8, 0, 4, new Color8Bit(235, 137, 52));
    private TalonFX m_motor = new TalonFX(MainPivotConstants.LEADER_CAN_ID);
    private MotionMagicVoltage m_profileReq = new MotionMagicVoltage(0);
    private VoltageOut m_voltageReq = new VoltageOut(0);
    private StatusSignal<Angle> m_angleSig = m_motor.getPosition();
    private double m_setpointRotations;
  /** Creates a new MainPivotS. */
  public MainPivotS() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    MAIN_PIVOT.setAngle(Units.rotationsToDegrees(m_angleSig.getValueAsDouble()));
  }

  public void simulationPeriodic() {
    for (int i = 0; i < 2; i++) {
      var simState = m_motor.getSimState();
      simState.setSupplyVoltage(12);
      // simState.getMotorVoltage is counterclockwise negative
      double volts = simState.getMotorVoltage();
      
      m_pivotSim.setInput(NomadMathUtil.subtractkS(volts, MainPivotConstants.K_S) - MainPivotConstants.K_G * Math.cos(getAngleRadians()));
      m_pivotSim.update(0.01);
      var rotorPos = m_pivotSim.getAngleRads() * MainPivotConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION / (2 * Math.PI);
      var rotorVel = m_pivotSim.getVelocityRadPerSec() * MainPivotConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION / (2 * Math.PI);
      
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

  public void setAngleRadians(double angle) {
    m_setpointRotations = Units.radiansToRotations(angle);
    
    if(m_setpointRotations < Units.degreesToRotations(2) && getAngleRotations() < Units.degreesToRotations(10)){
      m_motor.setControl(m_voltageReq.withOutput(0));
    } else {
      m_motor.setControl(m_profileReq.withPosition(m_setpointRotations));
    }
  }

  public Command goTo(DoubleSupplier angleSupplier) {
    return run(() -> setAngleRadians(angleSupplier.getAsDouble()));
  }

  public Command CORALLVL1INAngle() {
    return goTo(()->MainPivotConstants.CORALLVL1IN);
  }

  public Command CORALLVL2INAngle() {
    return goTo(()->MainPivotConstants.CORALLVL2IN);
  }

  public Command CORALLVL3INAngle() {
    return goTo(()->MainPivotConstants.CORALLVL3IN);
  }

  public Command CORALLVL4INAngle() {
    return goTo(()->MainPivotConstants.CORALLVL4IN);
  }

  public Command CORALSTATIONIN1() {
    return goTo(()->MainPivotConstants.CORALSTATIONIN1);
  }

  public Command CORALSTATIONIN2() {
    return goTo(()->MainPivotConstants.CORALSTATIONIN2);
  }

  public Command ALGAELVL1IN() {
    return goTo(()->MainPivotConstants.ALGAELVL1IN);
  }

  public Command ALGAELVL2IN() {
    return goTo(()->MainPivotConstants.ALGAELVL2IN);
  }

  public Command ALGAEOUT() {
    return goTo(()->MainPivotConstants.ALGAEOUT);
  }

  public Command SCORE_ANGLE() {
    return goTo(()->MainPivotConstants.SCORE_ANGLE);
  }

  public Command HANDOFF_ANGLE() {
    return goTo(()->MainPivotConstants.HANDOFF_ANGLE);
  }


  public Command hold() {
    return sequence(
        runOnce(() -> setAngleRadians(getAngleRadians())),
        Commands.idle());
  }

}
