// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.elevator.RealElevatorS.ElevatorConstants;
import frc.robot.util.NomadMathUtil;

@Logged
public class MainPivotS extends SubsystemBase {
  public class MainPivotConstants {

    // [Things related to hardware] such as motor hard limits, can ids, pid constants, motor
    // rotations per arm rotation.

    public static final Angle CCW_LIMIT = Degrees.of(110);
    public static final Angle CW_LIMIT = Degrees.of(40);
    public static final double MOTOR_ROTATIONS_PER_ARM_ROTATION = 79.3651 * 14.0/9.0;
    // Units=volts/pivot rotation/s
    public static final double K_V = 12.0 / (100 / MOTOR_ROTATIONS_PER_ARM_ROTATION);
    public static final double K_A = 0.25 /*v/oldRot/s^2 */ * 9.0/14.0; /* newRot/oldRot */;
    public static final double CG_DIST = Units.inchesToMeters(10);
    public static final LinearSystem<N2, N1, N2> PLANT =
        LinearSystemId.identifyPositionSystem(
            Units.radiansToRotations(K_V), Units.radiansToRotations(K_A));

    // Pose

    public static Angle climbAngle = CW_LIMIT.plus(Degrees.of(3));

    // CAN IDs
    public static final int LEADER_CAN_ID = 30;
    public static final int FOLLOWER_CAN_ID = 31;
    public static final int OPPOSING1_CAN_ID = 32;
    public static final int OPPOSING2_CAN_ID = 33;

    public static final int CURRENT_LIMIT = 100;

    public static final double OUT_VOLTAGE = 0;
    public static final double IN_VOLTAGE = 0;

    public static final double K_G_RETRACTED = 0.36;
    public static final double K_G_EXTENDED = 0.36;
    public static final double K_S = 0.1;
    public static final double K_P = 70.0;
    // arm plus hand
    public static final Mass ARM_MASS = Pounds.of(16).plus(Pounds.of(13.9));
    public static final DCMotor GEARBOX = DCMotor.getKrakenX60(4);

    public static final double ENCODER_OFFSET_ROTATIONS = 0.12109375 + 0.125;

    public static TalonFXConfiguration configureLeader(TalonFXConfiguration config) {
      config.Slot0.withKS(K_S).withKV(K_V).withKA(K_A).withKP(K_P).withKD(0.4).withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
      //TEMP
      config.MotionMagic.withMotionMagicCruiseVelocity(0.63).withMotionMagicAcceleration(0.82);//0.5, 0.66
      //config.MotionMagic.withMotionMagicCruiseVelocity(1).withMotionMagicAcceleration(4);
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.Feedback
          .withFeedbackRemoteSensorID(30)
          .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
          .withSensorToMechanismRatio(1)
          .withRotorToSensorRatio(MOTOR_ROTATIONS_PER_ARM_ROTATION);
      config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(CCW_LIMIT)
          .withReverseSoftLimitThreshold(Degrees.of(1.34))
          .withReverseSoftLimitEnable(true);
         // config.Voltage.withPeakForwardVoltage(10).withPeakReverseVoltage(-10);
      config.CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(40);
      return config;
    }

    public static TalonFXConfiguration configureFollower(TalonFXConfiguration config) {
      config.Feedback
          .withSensorToMechanismRatio(MOTOR_ROTATIONS_PER_ARM_ROTATION);
          config.CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(40);
      return config;
    }

    public static TalonFXConfiguration configureOppose(TalonFXConfiguration config) {
      config.Feedback
          .withSensorToMechanismRatio(-MOTOR_ROTATIONS_PER_ARM_ROTATION);
          config.CurrentLimits.withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(40);
      return config;
    }

    public static CANcoderConfiguration configureEncoder(CANcoderConfiguration config) {
      config.MagnetSensor.MagnetOffset = ENCODER_OFFSET_ROTATIONS;
      config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = Units.degreesToRotations(250);
      config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      return config;
    }
  }

  private final VariableLengthArmSim m_pivotSim =
      new VariableLengthArmSim(
          MainPivotConstants.PLANT,
          MainPivotConstants.GEARBOX,
          MainPivotConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION,
          ElevatorConstants.getMoI(ElevatorConstants.MIN_LENGTH.in(Meters)),
          ElevatorConstants.MIN_LENGTH.in(Meters),
          0,
          MainPivotConstants.CCW_LIMIT.in(Radians),
          MainPivotConstants.ARM_MASS.in(Kilograms),
          false);

  private DoubleSupplier m_lengthSupplier = () -> ElevatorConstants.MIN_LENGTH.in(Meters);
  private DoubleSupplier m_moiSupplier = () -> ElevatorConstants.getMoI(ElevatorConstants.MIN_LENGTH.in(Meters));
  private DoubleSupplier m_accelerationSupplier = () -> 0;
  public void setLengthSupplier(DoubleSupplier lengthSupplier) {
    m_lengthSupplier = lengthSupplier;
  }

  public void setMoISupplier(DoubleSupplier moiSupplier) {
    m_moiSupplier = moiSupplier;
  }

  public void setAccelerationSupplier(DoubleSupplier accelerationSupplier) {
    m_accelerationSupplier = accelerationSupplier;
  }

  public final MechanismLigament2d MAIN_PIVOT =
      new MechanismLigament2d("main_pivot", 0, 0, 4, new Color8Bit(235, 137, 52));
  private TalonFX m_leader = new TalonFX(MainPivotConstants.LEADER_CAN_ID, Robot.m_notSwerveBus);
  private TalonFX m_follower = new TalonFX(MainPivotConstants.FOLLOWER_CAN_ID, Robot.m_notSwerveBus);
  private TalonFX m_oppose1 = new TalonFX(MainPivotConstants.OPPOSING1_CAN_ID, Robot.m_notSwerveBus);
  private TalonFX m_oppose2 = new TalonFX(MainPivotConstants.OPPOSING2_CAN_ID, Robot.m_notSwerveBus);
  private MotionMagicVoltage m_profileReq = new MotionMagicVoltage(0);
  private VoltageOut m_voltageReq = new VoltageOut(0);
  private StatusSignal<Angle> m_angleSig = m_leader.getPosition();
  private StatusSignal<Double> m_angleSetpointSig = m_leader.getClosedLoopReference();

  private double m_goalRotations;
  private CANcoder m_cancoder = new CANcoder(30, Robot.m_notSwerveBus);
  private StatusSignal<Angle> m_cancoderAngleSig = m_cancoder.getPosition();
  /** Creates a new MainPivotS. */
  public MainPivotS() {
    m_leader
        .getConfigurator()
        .apply(MainPivotConstants.configureLeader(new TalonFXConfiguration()));
    m_follower
        .getConfigurator()
        .apply(MainPivotConstants.configureFollower(new TalonFXConfiguration()));
    m_oppose1
        .getConfigurator()
        .apply(MainPivotConstants.configureOppose(new TalonFXConfiguration()));
    m_oppose2
        .getConfigurator()
        .apply(MainPivotConstants.configureOppose(new TalonFXConfiguration()));
    m_cancoder.getConfigurator()
        .apply(MainPivotConstants.configureEncoder(new CANcoderConfiguration()));
    m_leader.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    m_follower.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    m_oppose1.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    m_oppose2.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    m_cancoder.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    m_pivotSim.setState(VecBuilder.fill(MainPivotConstants.CW_LIMIT.in(Radians), 0));

    m_follower.setControl(new Follower(MainPivotConstants.LEADER_CAN_ID, false));
    m_oppose1.setControl(new Follower(MainPivotConstants.LEADER_CAN_ID, true));
    m_oppose2.setControl(new Follower(MainPivotConstants.LEADER_CAN_ID, true));
    m_angleSetpointSig.setUpdateFrequency(50);
    setDefaultCommand(hold());
    setNeutralMode(NeutralModeValue.Brake);
  }

  public Command coast() {
    return this.startEnd(
      ()->setNeutralMode(NeutralModeValue.Coast), ()->setNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true);
  }

  public double getSetpoint() {
    m_angleSetpointSig.refresh();
    return m_angleSetpointSig.getValueAsDouble();
  }
  public double getCanCoderAngle() {
    m_cancoderAngleSig.refresh();
    return m_cancoderAngleSig.getValueAsDouble();
  }
  private void setNeutralMode(NeutralModeValue mode) {
    m_leader.setNeutralMode(mode);
    m_follower.setNeutralMode(mode);
    m_oppose1.setNeutralMode(mode);
    m_oppose2.setNeutralMode(mode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    MAIN_PIVOT.setAngle(Units.rotationsToDegrees(m_angleSig.getValueAsDouble()));
    
        if (DriverStation.isDisabled()) {
      m_leader.set(0);
    }
  }

  public double simulatedAngleRotations() {
    return Units.radiansToRotations(m_pivotSim.getAngleRads());
  }
  public void simulationPeriodic() {
    m_pivotSim.setMOI(m_moiSupplier.getAsDouble());
    m_pivotSim.setCGRadius(m_lengthSupplier.getAsDouble() / 2.0);
    for (int i = 0; i < 2; i++) {
      var simState = m_leader.getSimState();
      simState.setSupplyVoltage(12);
      // simState.getMotorVoltage is counterclockwise negative
      double volts = simState.getMotorVoltage();

      m_pivotSim.setInput(
          NomadMathUtil.subtractkS(volts, MainPivotConstants.K_S)
          // - getKgVolts()
          );
      m_pivotSim.update(0.01);
      var rotorPos =
          m_pivotSim.getAngleRads()
              * MainPivotConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION
              / (2 * Math.PI);
      var rotorVel =
          m_pivotSim.getVelocityRadPerSec()
              * MainPivotConstants.MOTOR_ROTATIONS_PER_ARM_ROTATION
              / (2 * Math.PI);

      simState.setRawRotorPosition(rotorPos);
      simState.setRotorVelocity(rotorVel);
      m_follower.getSimState().setRawRotorPosition(rotorPos);
      m_follower.getSimState().setRotorVelocity(rotorVel);
      m_oppose1.getSimState().setRawRotorPosition(-rotorPos);
      m_oppose1.getSimState().setRotorVelocity(-rotorVel);
      m_oppose2.getSimState().setRawRotorPosition(-rotorPos);
      m_oppose2.getSimState().setRotorVelocity(-rotorVel);
      m_cancoder.getSimState().setRawPosition(-(Units.radiansToRotations(m_pivotSim.getAngleRads())-MainPivotConstants.ENCODER_OFFSET_ROTATIONS));
      m_cancoder.getSimState().setVelocity(-(Units.radiansToRotations(m_pivotSim.getVelocityRadPerSec())));

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

  public double getLengthKg() {
    return MathUtil.interpolate(
      MainPivotConstants.K_G_RETRACTED,
      MainPivotConstants.K_G_EXTENDED,
      MathUtil.inverseInterpolate(
          ElevatorConstants.MIN_LENGTH.in(Meters),
          ElevatorConstants.MAX_LENGTH.in(Meters),
          getLengthMeters()));
  }
  public double getKgVolts() {
    return Math.cos(getAngleRadians())
        * getLengthKg();
  }

  //public double getDrivebasePivotAcceleration(double db_vx, double db_ax, double cg_dist, double moi_piv) {

    //A_drive x_drive + B_drive u_drive = [db_vx db_ax]
    //A_arm x_arm + B_arm u_arm = [arm_v arm_a]

    // u_arm = B_arm^-1 ([arm_v arm_a]-A_arm x_arm)

    // var cg_dist = m_lengthSupplier.getAsDouble() / 2.0;
    // var fx = 
    // //moi around pivot = moi_cg * cg_dist^2
    // var moi_cg = m_moiSupplier.getAsDouble() / (cg_dist*cg_dist);
    // var torque_around_cg = 
    // var alpha_rad_per_s = ax_base * cg_dist; // radians/second;
    // return Units.radiansToRotations(alpha_rad_per_s);

    /*
      [ cos th           l    ]  [ax]  = [-g sin theta]
      [ m1+m2      m_2l cos th]  [ath]  = F+m_2 l vth^2 sin th]

     */
    // var l = getLengthMeters();
    // var m2 = Units.lbsToKilograms(13.2);
    // var m1 = Units.lbsToKilograms(50);
    // var sin_th = Math.sin(getAngleRadians());
    // var cos_th = Math.cos(getAngleRadians());
    // var alpha = (-db_ax*cos_th)/l;
    //Math.cos(getAngleRadians())*db_ax + alpha*l = -g*Math.sin(getAngleRadians());
  //}

  public void setAngleRadians(double angle) {
    m_goalRotations = Units.radiansToRotations(angle);

    m_leader.setControl(
        m_profileReq.withPosition(m_goalRotations).withFeedForward(getKgVolts()));
  }

  public Command goTo(DoubleSupplier angleSupplier) {
    return run(() -> setAngleRadians(angleSupplier.getAsDouble()));
  }

  public Command goTo(Supplier<Angle> angleSupplier) {
    return run(() -> setAngleRadians(angleSupplier.get().in(Radians)));
  }

  public Command voltage(DoubleSupplier voltageSupplier) {
    return run(
        () -> {
          m_leader.setControl(m_voltageReq.withOutput(voltageSupplier.getAsDouble()));
        });
  }

  public Command hold() {
    return sequence(runOnce(() -> m_goalRotations = getAngleRotations()),
      run(()->{
        m_leader.setControl(
          m_profileReq.withPosition(m_goalRotations).withFeedForward(getKgVolts()));
      }));
  }
}
