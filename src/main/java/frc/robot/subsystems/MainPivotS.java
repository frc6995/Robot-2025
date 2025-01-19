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
import frc.robot.Robot;

public class MainPivotS extends SubsystemBase {
  public class MainPivotConstants {
    //[Things related to hardware] such as motor hard limits, can ids, pid constants, motor rotations per arm rotation.

    public static final double CCW_LIMIT = Units.degreesToRadians(100);
    public static final double CW_LIMIT = Units.degreesToRadians(40);
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
    public static final int CAN_ID = 42;
    public static final double HANDOFF_ANGLE = MainPivotS.MainPivotConstants.CCW_LIMIT;

    public static final double MOTOR_ROTATIONS_PER_ARM_ROTATION = 70;
  }

  public final MechanismLigament2d MAIN_PIVOT = new MechanismLigament2d(
    "main_pivot", 8, 0, 4, new Color8Bit(235, 137, 52));
  /** Creates a new MainPivotS. */
  public MainPivotS() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void simulationPeriodic() {

  }

  public Command goTo(double armRotations) {
    return none();
  }

  public Command hold() {
    return none();
  }


}
