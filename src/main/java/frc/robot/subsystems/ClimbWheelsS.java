// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
@Logged
public class ClimbWheelsS extends SubsystemBase {
  public class ClimbWheelsConstants {
    public static final int CAN_ID = 55;
    public static final int CURRENT_LIMIT = 40;
    public static TalonFXConfiguration configuremotor(TalonFXConfiguration config){
      config.CurrentLimits
        .withStatorCurrentLimit(CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        ;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      return config;
    }
  
  }
  public Command in(){
    return voltage(-6);
  }
  public Command out(){
    return voltage(1);
  }
  public Command stop(){
    return voltage(0);
  }
  private Command voltage(double volts){
    return this.run(() -> motor.setControl(voltageReq.withOutput(volts)));
  }
  
  /** Creates a new ClimbHookS. */
  public ClimbWheelsS() {
    motor.getConfigurator().apply(ClimbWheelsConstants.configuremotor(new TalonFXConfiguration()));
    setDefaultCommand(stop());
  }

  public Command coast() {
    return this.startEnd(
      ()->setNeutralMode(NeutralModeValue.Coast), ()->setNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true);
  }

  private void setNeutralMode(NeutralModeValue value) {
    motor.setNeutralMode(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public final TalonFX motor = new TalonFX(ClimbWheelsConstants.CAN_ID);
  private final VoltageOut voltageReq = new VoltageOut(0);
}
