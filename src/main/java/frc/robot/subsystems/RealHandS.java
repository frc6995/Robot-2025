// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaePivotS.AlgaePivotConstants;

@Logged
public class RealHandS extends Hand {
    public final MechanismLigament2d TOP_ROLLER =
      new MechanismLigament2d(
          "top-roller", 0.05, 0, 4, new Color8Bit(Color.kWhite));
  public final MechanismLigament2d BOTTOM_ROLLER =
      new MechanismLigament2d(
          "top-roller", 0.05, 0, 4, new Color8Bit(Color.kWhite));
  public class HandConstants {

  public static final int CAN_ID = 51;

  public static final double IN_CORAL_VOLTAGE = 5;

  public static final double OUT_CORAL_VOLTAGE = -4.5; //worked with -6 but coral bounced

  public static final double IN_ALGAE_VOLTAGE = -8;

  public static final double OUT_ALGAE_VOLTAGE = 10;

  public static TalonFXConfiguration configureMotor(TalonFXConfiguration config) {
    config.CurrentLimits.withStatorCurrentLimit(70);
    return config;
  }
  }
  private final TalonFX motor = new TalonFX(HandConstants.CAN_ID);

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final DCMotorSim motorSim = new DCMotorSim(
    LinearSystemId.identifyPositionSystem(Units.radiansToRotations(0.12), Units.radiansToRotations(0.003)), DCMotor.getKrakenX60(1));
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





  /** Creates a new HandRollerS. */
  public RealHandS() {
   super();
   motor.getConfigurator().apply(HandConstants.configureMotor(new TalonFXConfiguration()));
   setDefaultCommand(stop());

  }

  @Override
  public void periodic() {
    TOP_ROLLER.setAngle(TOP_ROLLER.getAngle() + 4 * voltageRequest.Output);
    BOTTOM_ROLLER.setAngle(BOTTOM_ROLLER.getAngle() - 4 * voltageRequest.Output);
    // This method will be called once per scheduler run
  }
  private Command voltage(double voltage) {
    return this.run(()->motor.setControl(voltageRequest.withOutput(voltage)));
  }
  public Command stop(){
    return voltage(0);
  }
  public Command inCoral(){
    return voltage(HandConstants.IN_CORAL_VOLTAGE);
  }
  public Command outCoral(){
    return voltage(HandConstants.OUT_CORAL_VOLTAGE);

  }

  @Override
  public Command inAlgae() {
    return voltage(HandConstants.IN_ALGAE_VOLTAGE);
  }





  @Override
  public Command outAlgae() {
    return voltage(HandConstants.OUT_ALGAE_VOLTAGE);
  }

}
