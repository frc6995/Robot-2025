// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmBrakeS extends SubsystemBase {
  public final SparkFlex motor = new SparkFlex(ArmBrakeConstants.CAN_ID, MotorType.kBrushless); 
  /** Creates a new ArmBreakS. */
  public ArmBrakeS() {
    motor.configure(ArmBrakeConstants.configureMotor(new SparkFlexConfig()), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command brake() {
return voltage(-1);
  }
  public Command release() {
return voltage(1);
  }
  public Command stop () {
return voltage(0);
  }
  public Command voltage (double volts) {
return this.run(()-> motor.setVoltage(volts));

  }

public class ArmBrakeConstants {
  public static final int CAN_ID = 35;
  public static final int CURRENT_LIMIT = 5;
  public static SparkFlexConfig configureMotor(SparkFlexConfig config) {
    config.smartCurrentLimit(CURRENT_LIMIT);
    return config;
  }
}
}
