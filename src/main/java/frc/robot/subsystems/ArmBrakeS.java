// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ArmBrakeS extends SubsystemBase {
  public final SparkFlex motor = new SparkFlex(ArmBrakeConstants.CAN_ID, MotorType.kBrushless); 
  /** Creates a new ArmBreakS. */
  public ArmBrakeS() {
    motor.configure(ArmBrakeConstants.configureMotor(new SparkFlexConfig()), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    setDefaultCommand(release());//.andThen(holdopen()));
  }
  public double getPosition() {
    return motor.getEncoder().getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public Command start() {
    return voltage (10).withTimeout(0.06);
  }
  public Command goTo(double position) {
    return this.run(()->motor.getClosedLoopController().setReference(position, ControlType.kPosition));
  }
  public Command brake() {  
  return this.goTo(-0.01);//return voltage(-1);   
  }
  public Command release() {
    return goTo(0.03); //start().andThen(end());
  }
  public Command holdopen() {
return voltage(0.2);
  }
  public Command end () {
return voltage(0);
  }
  public Command voltage (double volts) {
return this.run(()-> motor.setVoltage(volts));

  }

  public Command home () {
    return this.runOnce(()->motor.getEncoder().setPosition(0)).ignoringDisable(true);
  }
public class ArmBrakeConstants {
  public static final int CAN_ID = 35;
  public static final int CURRENT_LIMIT = 15;
  public static SparkFlexConfig configureMotor(SparkFlexConfig config) {
    config.smartCurrentLimit(CURRENT_LIMIT);
    config.softLimit.reverseSoftLimit(-1).reverseSoftLimitEnabled(true);
    config.idleMode(IdleMode.kBrake);
    config.closedLoop.p(10 );
    return config;
  }
}
}