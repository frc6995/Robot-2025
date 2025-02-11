// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ClimbHookS extends SubsystemBase {
  public class ClimbHookConstants {
    public static final int CAN_ID = 54;
    public static final int CURRENT_LIMIT = 5;
    public static final double OUT_VOLTAGE = 0;
    public static final double IN_VOLTAGE = 0;
    public static TalonFXConfiguration configuremotor(TalonFXConfiguration config){
      config.CurrentLimits.withStatorCurrentLimit(CURRENT_LIMIT).withStatorCurrentLimitEnable(false);
      return config;
    }
  
  }
  public Command clamp(){
    return voltage(12);
  }
  public Command release(){
    return voltage(-12);
  }
  public Command stop(){
    return voltage(0);
  }
  private Command voltage(double volts){
    return this.run(() -> motor.setControl(voltageReq.withOutput(volts)));
  }
  
  /** Creates a new ClimbHookS. */
  public ClimbHookS() {
    motor.getConfigurator().apply(ClimbHookConstants.configuremotor(new TalonFXConfiguration()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public final TalonFX motor = new TalonFX(ClimbHookConstants.CAN_ID);
  private final VoltageOut voltageReq = new VoltageOut(0);
}
