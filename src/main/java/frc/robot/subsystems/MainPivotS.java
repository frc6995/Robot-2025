// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/* Changed the imports to be compatible */
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.NomadMathUtil;
import frc.robot.Robot;
import frc.robot.subsystems.ElevatorS.ElevatorConstants;

@Logged
public class MainPivotS extends SubsystemBase {
  public class MainPivotConstants {

    //[Things related to hardware] such as motor hard limits, can ids, pid constants, motor rotations per arm rotation.

    public static final Angle CCW_LIMIT = Degrees.of(100);
    public static final Angle CW_LIMIT = Degrees.of(40);
    public static final double MOTOR_ROTATIONS_PER_ARM_ROTATION = 70;
    // Units=volts/pivot rotation/s
    public static final double K_V = 9.2;
    public static final double K_A = 0.04;
    public static final double CG_DIST = Units.inchesToMeters(10);
    public static final LinearSystem<N2, N1, N2> PLANT = LinearSystemId
        .identifyPositionSystem(Units.radiansToRotations(K_V), Units.radiansToRotations(K_A));

    //Pose

    //CAN IDs
    public static final int LEADER_CAN_ID = 30;
    public static final int FOLLOWER_CAN_ID = 31;
    public static final int OPPOSING1_CAN_ID = 32;
    public static final int OPPOSING2_CAN_ID = 33;

    public static final int CURRENT_LIMIT = 100;
   
    public static final double OUT_VOLTAGE = 0;
    public static final double IN_VOLTAGE = 0;



    public static final double K_G_RETRACTED = 0.5;
    public static final double K_G_EXTENDED = 3;
    public static final double K_S = 0;
    // arm plus hand
    public static final Mass ARM_MASS = Pounds.of(16).plus(Pounds.of(0));
    public static final DCMotor GEARBOX = DCMotor.getKrakenX60(4);
    public static final InterpolatingDoubleTreeMap LENGTH_TO_MOI = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(ElevatorConstants.MIN_LENGTH.in(Meters), 0.7419),
      Map.entry(ElevatorConstants.MAX_LENGTH.in(Meters), 2.812027)
    );
    public static double getMoI(double armLengthMeters) {
      return LENGTH_TO_MOI.get(armLengthMeters);
    }
    public static TalonFXConfiguration configureLeader(TalonFXConfiguration config) {
      config.Slot0
        .withKS(K_S).withKV(K_V).withKA(K_A)
        .withKP(10).withKD(1);
      config.MotionMagic
        .withMotionMagicCruiseVelocity(0.5)
        .withMotionMagicAcceleration(2);
      config.Feedback
        // .withFeedbackRemoteSensorID(34)
        // .withFeedbackSensorSource(FeedbackSensorSourceValue.SyncCANcoder)
        .withSensorToMechanismRatio(MOTOR_ROTATIONS_PER_ARM_ROTATION);
      config.SoftwareLimitSwitch
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(CCW_LIMIT)
        .withReverseSoftLimitThreshold(CW_LIMIT)
        .withReverseSoftLimitEnable(true);
      return config;
    }
    public static TalonFXConfiguration configureFollower(TalonFXConfiguration config) {
      config.Feedback
        // .withFeedbackRemoteSensorID(34)
        // .withFeedbackSensorSource(FeedbackSensorSourceValue.SyncCANcoder)
        .withSensorToMechanismRatio(MOTOR_ROTATIONS_PER_ARM_ROTATION);
      return config;
    }
    public static TalonFXConfiguration configureOppose(TalonFXConfiguration config) {
      config.Feedback
        // .withFeedbackRemoteSensorID(34)
        // .withFeedbackSensorSource(FeedbackSensorSourceValue.SyncCANcoder)
        .withSensorToMechanismRatio(-MOTOR_ROTATIONS_PER_ARM_ROTATION)
        ;
      return config;
    }
  }

  private final VariableLengthArmSim m_pivotSim = new VariableLengthArmSim(
      MainPivotConstants.PLANT,
      MainPivotConstants.GEARBOX,
      MainPivotConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION,
      MainPivotConstants.getMoI(ElevatorConstants.MIN_LENGTH.in(Meters)),
      ElevatorConstants.MIN_LENGTH.in(Meters), 
      MainPivotConstants.CW_LIMIT.in(Radians),
      MainPivotConstants.CCW_LIMIT.in(Radians),
      
      MainPivotConstants.ARM_MASS.in(Kilograms),
      false);

  private DoubleSupplier m_lengthSupplier = ()->ElevatorConstants.MIN_LENGTH.in(Meters);
  public void setLengthSupplier(DoubleSupplier lengthSupplier) {
    m_lengthSupplier = lengthSupplier;
  }
  public final MechanismLigament2d MAIN_PIVOT = new MechanismLigament2d(
    "main_pivot", 0, 0, 4, new Color8Bit(235, 137, 52));
    private TalonFX m_leader = new TalonFX(MainPivotConstants.LEADER_CAN_ID);
    private TalonFX m_follower = new TalonFX(MainPivotConstants.FOLLOWER_CAN_ID);
    private TalonFX m_oppose1 = new TalonFX(MainPivotConstants.OPPOSING1_CAN_ID);
    private TalonFX m_oppose2 = new TalonFX(MainPivotConstants.OPPOSING2_CAN_ID);
    private MotionMagicVoltage m_profileReq = new MotionMagicVoltage(0);
    private VoltageOut m_voltageReq = new VoltageOut(0);
    private StatusSignal<Angle> m_angleSig = m_leader.getPosition();
    private double m_setpointRotations;
  /** Creates a new MainPivotS. */
  public MainPivotS() {
    m_leader.getConfigurator().apply(MainPivotConstants.configureLeader(new TalonFXConfiguration()));
    m_follower.getConfigurator().apply(MainPivotConstants.configureFollower(new TalonFXConfiguration()));
    m_oppose1 .getConfigurator().apply(MainPivotConstants.configureOppose(new TalonFXConfiguration()));
    m_oppose2 .getConfigurator().apply(MainPivotConstants.configureOppose(new TalonFXConfiguration()));
    m_leader.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    m_follower.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    m_oppose1.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    m_oppose2.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    m_pivotSim.setState(VecBuilder.fill(MainPivotConstants.CW_LIMIT.in(Radians), 0));
    m_follower.setControl(new Follower(MainPivotConstants.LEADER_CAN_ID, false));
    m_oppose1.setControl(new Follower(MainPivotConstants.LEADER_CAN_ID, true));
    m_oppose2.setControl(new Follower(MainPivotConstants.LEADER_CAN_ID, true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    MAIN_PIVOT.setAngle(Units.rotationsToDegrees(m_angleSig.getValueAsDouble()));
  }

  public void simulationPeriodic() {
    m_pivotSim.setMOI(MainPivotConstants.getMoI(m_lengthSupplier.getAsDouble()));
    m_pivotSim.setCGRadius(m_lengthSupplier.getAsDouble() / 2.0);
    for (int i = 0; i < 2; i++) {
      var simState = m_leader.getSimState();
      simState.setSupplyVoltage(12);
      // simState.getMotorVoltage is counterclockwise negative
      double volts = simState.getMotorVoltage();
      
      m_pivotSim.setInput(NomadMathUtil.subtractkS(volts, MainPivotConstants.K_S)
      // - getKgVolts()
      );
      m_pivotSim.update(0.01);
      var rotorPos = m_pivotSim.getAngleRads() * MainPivotConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION / (2 * Math.PI);
      var rotorVel = m_pivotSim.getVelocityRadPerSec() * MainPivotConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION / (2 * Math.PI);
      
      simState.setRawRotorPosition(rotorPos);
      simState.setRotorVelocity(rotorVel);
      m_follower.getSimState().setRawRotorPosition(rotorPos);
      m_follower.getSimState().setRotorVelocity(rotorVel);
      m_oppose1.getSimState().setRawRotorPosition(-rotorPos);
      m_oppose1.getSimState().setRotorVelocity(-rotorVel);
      m_oppose2.getSimState().setRawRotorPosition(-rotorPos);
      m_oppose2.getSimState().setRotorVelocity(-rotorVel);
    }
  }

  public double getAngleRotations() {
    return m_angleSig.getValueAsDouble();
  }

  public double getAngleRadians() {
    return Units.rotationsToRadians(m_angleSig.getValueAsDouble());
  }
  private double getLengthMeters() {
    return m_lengthSupplier.getAsDouble();
  }
  public double getKgVolts() {
    return Math.cos(getAngleRadians()) *
     MathUtil.interpolate(
      MainPivotConstants.K_G_RETRACTED, MainPivotConstants.K_G_EXTENDED,
      MathUtil.inverseInterpolate(
        ElevatorConstants.MIN_LENGTH.in(Meters), ElevatorConstants.MAX_LENGTH.in(Meters), getLengthMeters()));
  }
  public void setAngleRadians(double angle) {
    m_setpointRotations = Units.radiansToRotations(angle);
    
    m_leader.setControl(m_profileReq.withPosition(m_setpointRotations));//.withFeedForward(getKgVolts()));
  }

  public Command goTo(DoubleSupplier angleSupplier) {
    return run(() -> setAngleRadians(angleSupplier.getAsDouble()));
  }
  public Command goTo(Supplier<Angle> angleSupplier) {
    return run(() -> setAngleRadians(angleSupplier.get().in(Radians)));
  }
  public Command voltage(DoubleSupplier voltageSupplier) {
    return run(()->{
      m_leader.setControl(m_voltageReq.withOutput(voltageSupplier.getAsDouble()));
    });
  }

  public Command hold() {
    return sequence(
        runOnce(() -> setAngleRadians(getAngleRadians())),
        Commands.idle());
  }

}
