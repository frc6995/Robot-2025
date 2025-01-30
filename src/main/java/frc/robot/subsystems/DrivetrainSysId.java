package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DrivetrainSysId {
  private DriveBaseS m_drivetrain;

  public DrivetrainSysId(DriveBaseS drive) {
    m_drivetrain = drive;

    m_sysIdRoutineTranslation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default rVolt rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with SignalLogger class
                state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> m_drivetrain.setControl(m_translationCharacterization.withVolts(output)),
                null,
                m_drivetrain));
    m_sysIdRoutineSteer =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default rVolt rate (1 V/s)
                Volts.of(7), // Use dynamic voltage of 7 V
                null, // Use default timeout (10 s)
                // Log state with SignalLogger class
                state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> m_drivetrain.setControl(m_steerCharacterization.withVolts(volts)),
                null,
                m_drivetrain));
    m_sysIdRoutineRotation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                /* This is in radians per second², but SysId only supports "volts per second" */
                Volts.of(Math.PI / 6).per(Second),
                /* This is in radians per second, but SysId only supports "volts" */
                Volts.of(Math.PI),
                null, // Use default timeout (10 s)
                // Log state with SignalLogger class
                state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> {
                  /* output is actually radians per second, but SysId only supports "volts" */
                  m_drivetrain.setControl(
                      m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                  /* also log the requested output for SysId */
                  SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                },
                null,
                m_drivetrain));
  }

  /* SYSID */

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation;

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer;

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation;

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdTranslationQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineTranslation.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdTranslationDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineTranslation.dynamic(direction);
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdRotationQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRotation.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdRotationDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRotation.dynamic(direction);
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineSteer.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineSteer.dynamic(direction);
  }

  public record SysIdConstants(
      Voltage ANGULAR_KS,
      Per<VoltageUnit, AngularVelocityUnit> ANGULAR_KV,
      Per<VoltageUnit, AngularAccelerationUnit> ANGULAR_KA,
      Voltage LINEAR_KS,
      Per<VoltageUnit, AngularVelocityUnit> LINEAR_KV,
      Per<VoltageUnit, AngularAccelerationUnit> LINEAR_KA,
      Voltage STEER_KS,
      Per<VoltageUnit, AngularVelocityUnit> STEER_KV,
      Per<VoltageUnit, AngularAccelerationUnit> STEER_KA) {}

  public static final PerUnit<VoltageUnit, AngularAccelerationUnit>
      VoltsPerRotationPerSecondSquared = Volts.per(RotationsPerSecondPerSecond);
  public static final PerUnit<VoltageUnit, AngularVelocityUnit> VoltsPerRotationPerSecond =
      Volts.per(RotationsPerSecond);
  public static final SysIdConstants SimSysIdConstants =
      new SysIdConstants(

          /*
           * Calculation
           *
           * 1. Run all tests once
           * 2. Convert the hoot log into a wpilog
           * 3. Load the wpilog into SysID
           *
           *
           * 4. Load a test with SysIdRotation_State, TalonFX-12/Velocity, TalonFX-12/Position, TalonFX-12/MotorVoltage
           * 5. Copy the values into the below
           */
          // ANGULAR_KS
          Volts.of(0.20174),
          // ANGULAR_KV
          VoltsPerRotationPerSecond.ofNative(0.12362),
          // ANGULAR_KA
          VoltsPerRotationPerSecondSquared.ofNative(0.002584),
          /*
           * 6. Load a test with SysIdTranslation_State, TalonFX-12/Velocity, TalonFX-12/Position, TalonFX-12/MotorVoltage
           * 7. Copy below
           */
          // LINEAR_KS
          Volts.of(0.20137),
          // LINEAR_KV
          VoltsPerRotationPerSecond.ofNative(0.12362),
          // LINEAR_KA
          VoltsPerRotationPerSecondSquared.ofNative(0.002713),
          /*
           * 8. Load a test with SysIdSteer_State, TalonFX-11/Velocity, TalonFX-11/Position, TalonFX-11/MotorVoltage
           */
          // STEER_KS
          Volts.of(0.19643),
          // STEER_KV
          VoltsPerRotationPerSecond.ofNative(2.6486),
          // STEER_KA
          VoltsPerRotationPerSecondSquared.ofNative(0.018463));

  public static final SysIdConstants AlphaSysIdConstants =
      new SysIdConstants(

          /*
           * Calculation
           *
           * 1. Run all tests once
           * 2. Convert the hoot log into a wpilog
           * 3. Load the wpilog into SysID
           *
           *
           * 4. Load a test with SysIdRotation_State, TalonFX-12/Velocity, TalonFX-12/Position, TalonFX-12/MotorVoltage
           * 5. Copy the values into the below
           */
          // ANGULAR_KS
          Volts.of(0.1),
          // ANGULAR_KV
          VoltsPerRotationPerSecond.ofNative(0.12362),
          // ANGULAR_KA
          VoltsPerRotationPerSecondSquared.ofNative(0.000),
          /*
           * 6. Load a test with SysIdTranslation_State, TalonFX-12/Velocity, TalonFX-12/Position, TalonFX-12/MotorVoltage
           * 7. Copy below
           */
          // LINEAR_KS
          Volts.of(0.13342),
          // LINEAR_KV
          VoltsPerRotationPerSecond.ofNative(0.12551 * 0.8 / 0.78),
          // LINEAR_KA
          VoltsPerRotationPerSecondSquared.ofNative(0.0054729),
          /*
           * 8. Load a test with SysIdSteer_State, TalonFX-11/Velocity, TalonFX-11/Position, TalonFX-11/MotorVoltage
           */
          // STEER_KS
          Volts.of(0.052455),
          // STEER_KV
          VoltsPerRotationPerSecond.ofNative(2.655),
          // STEER_KA
          VoltsPerRotationPerSecondSquared.ofNative(0.11018));
}
