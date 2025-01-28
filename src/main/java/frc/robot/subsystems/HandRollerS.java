// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaePivotS.AlgaePivotConstants;

public class HandRollerS extends SubsystemBase {
  public class HandRollerSconstants {

  public static final int CAN_ID = 51;

  public static final double IN_VOLTAGE = 6;

  public static final double OUT_VOLTAGE = -6;

  }
  private final TalonFX motor = new TalonFX(HandRollerSconstants.CAN_ID);

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
  public HandRollerS() {
    setDefaultCommand(stop());
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
  public Command stop(){
    return this.run(()->motor.setControl(voltageRequest.withOutput(0)));
  }
  public Command in(){
    return this.run(()->motor.setControl(voltageRequest.withOutput(HandRollerSconstants.IN_VOLTAGE)));
  }
  public Command out(){
    return this.run(()->motor.setControl(voltageRequest.withOutput(HandRollerSconstants.OUT_VOLTAGE)));

  }

}
