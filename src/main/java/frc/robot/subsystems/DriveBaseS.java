package frc.robot.subsystems;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.logging.PowerDistributionSim.Channel.c04_FL_Drive;
import static frc.robot.logging.PowerDistributionSim.Channel.c05_FL_Steer;
import static frc.robot.logging.PowerDistributionSim.Channel.c08_FR_Steer;
import static frc.robot.logging.PowerDistributionSim.Channel.c09_FR_Drive;
import static frc.robot.logging.PowerDistributionSim.Channel.c10_BR_Steer;
import static frc.robot.logging.PowerDistributionSim.Channel.c11_BR_Drive;
import static frc.robot.logging.PowerDistributionSim.Channel.c12_BR_Steer;
import static frc.robot.logging.PowerDistributionSim.Channel.c15_BL_Drive;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.logging.Module;
import frc.robot.logging.TalonFXPDHChannel;
import frc.robot.subsystems.drive.Pathing;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.RepulsorFieldPlanner;
import frc.robot.util.TrapezoidProfile;
import frc.robot.util.TrapezoidProfile.Constraints;
import frc.robot.util.TrapezoidProfile.State;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily
 * be used in command-based projects.
 */
@Logged
public class DriveBaseS extends TunerSwerveDrivetrain implements Subsystem {
  private RobotConfig m_robotConfig = new RobotConfig(
    Pounds.of(111 + 10 + 13), KilogramSquareMeters.of(6.0),
    new ModuleConfig(
      
      TunerConstants.kWheelRadius.in(Meters),
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), 1.5,
      DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.BackLeft.DriveMotorGearRatio),
      600.0, 1),
      getModuleLocations()
    );
  public com.pathplanner.lib.util.swerve.SwerveSetpointGenerator m_pathPlannerGenerator =
    new com.pathplanner.lib.util.swerve.SwerveSetpointGenerator(m_robotConfig,
      TunerConstants.FrontLeft.SteerMotorGains.kV * 12*2*Math.PI);
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during field-centric path following */
  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
      .withDriveRequestType(DriveRequestType.Velocity);

  private final PIDController m_pathXController = new PIDController(14, 0, 0.2);
  private final PIDController m_pathYController = new PIDController(14, 0, 0.2);
  private final PIDController m_pathThetaController = new PIDController(7, 0, 0);
  public final ProfiledPIDController m_profiledThetaController = new ProfiledPIDController(7, 0, 0,
      new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(8, 12));
  private final Vision m_vision = new Vision(this::addVisionMeasurement, () -> state().Pose);

  // public SwerveSetpointGenerator m_setpointGenerator = new SwerveSetpointGenerator(
  //   getModuleLocations(),
  //   new DCMotorExt(DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.BackLeft.DriveMotorGearRatio), 1),
  //         DCMotor.getKrakenX60(1).withReduction(TunerConstants.BackLeft.SteerMotorGearRatio),
  //   TunerConstants.kSlipCurrent.in(Amps),
  //   TunerConstants.kSlipCurrent.in(Amps),
  //   Units.lbsToKilograms(111 + 10 + 13),
  //   6.0, TunerConstants.kWheelRadius.in(Meters) * 2, 1.5, 0.0);
  public RepulsorFieldPlanner m_repulsor = new RepulsorFieldPlanner();
  // For logging
  public Module fl;
  public Module fr;
  public Module bl;
  public Module br;

  private Module makeModule(int idx) {
    return new Module(this.getModule(idx).getSteerMotor(), this.getModule(idx).getDriveMotor());
  }

  private void setupModuleLoggers() {
    fl = makeModule(0);
    fr = makeModule(1);
    bl = makeModule(2);
    br = makeModule(3);
    if (RobotBase.isSimulation()) {
      TalonFXPDHChannel.registerFD(c04_FL_Drive, fl.drive());
      TalonFXPDHChannel.registerFD(c05_FL_Steer, fl.steer());
      TalonFXPDHChannel.registerFD(c09_FR_Drive, fr.drive());
      TalonFXPDHChannel.registerFD(c08_FR_Steer, fr.steer());
      TalonFXPDHChannel.registerFD(c15_BL_Drive, bl.drive());
      TalonFXPDHChannel.registerFD(c12_BR_Steer, bl.steer());
      TalonFXPDHChannel.registerFD(c11_BR_Drive, br.drive());
      TalonFXPDHChannel.registerFD(c10_BR_Steer, br.steer());
    }
  }
  @NotLogged
  public SwerveDriveState state = getState();
  /** Re-expose the state as a method of the subclass so Epilogue finds it. */
  public SwerveDriveState state() {
    return state;
  }

  private final SwerveDrivetrainConstants constants;
  private final SwerveModuleConstants<?, ?, ?>[] modules;

  private double getOffset(int moduleIndex) {
    return Rotation2d.fromRotations(modules[moduleIndex].EncoderOffset)
        .minus(this.state().ModuleStates[moduleIndex].angle)
        .getRotations();
  }

  public double offsetFL() {
    return getOffset(0);
  }

  public double offsetFR() {
    return getOffset(1);
  }

  public double offsetBL() {
    return getOffset(2);
  }

  public double offsetBR() {
    return getOffset(3);
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>
   * This constructs the underlying hardware devices, so user should not construct
   * the devices
   * themselves. If they need the devices, they can access them through getters in
   * the classes.
   */
  public DriveBaseS(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    this.constants = drivetrainConstants;
    this.modules = modules;
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
    setupModuleLoggers();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public Pose2d targetPose() {
    return new Pose2d(
        m_pathXController.getSetpoint(),
        m_pathYController.getSetpoint(),
        Rotation2d.fromRadians(m_pathThetaController.getSetpoint()));
  }

  public Command repulsorCommand(Supplier<Pose2d> target) {
    return run(
        () -> {
          m_repulsor.setGoal(target.get().getTranslation());
          followPath(
              m_repulsor.getCmd(state().Pose, state().Speeds, 2, true, target.get().getRotation()));
        });
  }

  /**
   * Returns a command that applies the specified control request to this swerve
   * drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Follows the given field-centric path sample with PID.
   *
   * @param pose   Current pose of the robot
   * @param sample Sample along the path to follow
   */
  public void followPath(SwerveSample sample) {
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
    var pose = state().Pose;
    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(pose.getRotation().getRadians(),
        sample.heading);

    setControl(
        m_pathApplyFieldSpeeds
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  SwerveRequest.ApplyRobotSpeeds idle = new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds()).withDriveRequestType(DriveRequestType.Velocity);
  SwerveRequest.ApplyRobotSpeeds brakeMode = new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds()).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake().withDriveRequestType(DriveRequestType.Velocity);
  public Command stop() {
    return applyRequest(() -> idle);
  }
  public Command stopBrakeMode() {
    return applyRequest(() -> brakeMode);
  }

  private final SwerveSample[] emptyTrajectory = new SwerveSample[0];
  public SwerveSample[] currentTrajectory = emptyTrajectory;

  public void logTrajectory(Trajectory<SwerveSample> traj, boolean isStarting) {
    currentTrajectory = isStarting ? traj.samples().toArray(SwerveSample[]::new) : emptyTrajectory;
  }

  
  @Override
  public void periodic() {
    state = getStateCopy();
    if (RobotBase.isReal()) {
      
      m_vision.update();
    }
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled.
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
  }

  private void updateSimCurrents() {
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(
        () -> {
          final double currentTime = Utils.getCurrentTimeSeconds();
          double deltaTime = currentTime - m_lastSimTime;
          m_lastSimTime = currentTime;

          /* use the measured time delta, get battery voltage from WPILib */
          updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /* SYSID */

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  /*
   * SysId routine for characterizing translation. This is used to find PID gains
   * for the drive motors.
   */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /*
   * SysId routine for characterizing steer. This is used to find PID gains for
   * the steer motors.
   */
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(7), // Use dynamic voltage of 7 V
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle
   * HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
   * importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
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
            setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            /* also log the requested output for SysId */
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
          },
          null,
          this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine
   * specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified
   * by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  public void resetOdometry(Pose2d pose) {
    this.resetPose(pose);
    // m_vision.resetPose();
  }

  public Command driveToX(DoubleSupplier x, DoubleSupplier ySpeed, Supplier<Rotation2d> heading) {
    return applyRequest(() -> {
      var pose = state().Pose;
      var targetSpeeds = new ChassisSpeeds(
          m_pathXController.calculate(pose.getX(), x.getAsDouble()),
          (AllianceFlipUtil.shouldFlip() ? -1 : 1) *ySpeed.getAsDouble(),
          m_pathThetaController.calculate(
              pose.getRotation().getRadians(), heading.get().getRadians()));
      return m_pathApplyFieldSpeeds.withSpeeds(
          targetSpeeds).withWheelForceFeedforwardsX(emptyFeedForwards).withWheelForceFeedforwardsX(emptyFeedForwards);
    });
  }

  @NotLogged
  public Pose2d getPose() {
    return state.Pose;
  }

  @NotLogged
  public Rotation2d getPoseHeading() {
    return getPose().getRotation();
  }

  @NotLogged
  public ChassisSpeeds getFieldRelativeLinearSpeedsMPS() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(state().Speeds,state().Pose.getRotation());
  }

  private double[] emptyFeedForwards = new double[] { 0, 0, 0, 0 };

  public Command pidToPoseC(Supplier<Pose2d> poseSupplier) {
    return this.run(
        () -> {
          var target = poseSupplier.get();
          m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
          var pose = state().Pose;
          var targetSpeeds = new ChassisSpeeds(
              m_pathXController.calculate(pose.getX(), target.getX()),
              m_pathYController.calculate(pose.getY(), target.getY()),
              m_pathThetaController.calculate(
                  pose.getRotation().getRadians(), target.getRotation().getRadians()));
          setControl(m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
              .withWheelForceFeedforwardsX(emptyFeedForwards).withWheelForceFeedforwardsX(emptyFeedForwards));
        });
  }

  public Command pidToPoseC(Optional<Pose2d> poseOpt) {
    return poseOpt.map((pose) -> pidToPoseC(() -> pose)).orElse(Commands.none());
  }

  public Command pidToPoseC(Pose2d poseSup) {
    return pidToPoseC(() -> poseSup);
  }

  /* DRIVE TO POSE using Trapezoid Profiles */
  private TrapezoidProfile.Constraints driveToPoseConstraints = new Constraints(2, 1.5);
  private TrapezoidProfile.Constraints driveToPoseRotationConstraints = new Constraints(3, 6);
  private TrapezoidProfile driveToPoseProfile = new TrapezoidProfile(driveToPoseConstraints);
  private TrapezoidProfile driveToPoseRotationProfile = new TrapezoidProfile(driveToPoseRotationConstraints);

  // For modifying goals from within a lambda
  private class Capture<T> {
    public T inner;

    public Capture(T inner) {
      this.inner = inner;
    }
  }

  private TrapezoidProfile.State driveToPoseGoal = new State(0, 0);
  private TrapezoidProfile.State driveToPoseRotationGoal = new State(0, 0);


  private SwerveSetpoint m_previousSwerveSetpoint = new SwerveSetpoint(state().Speeds, state().ModuleStates, DriveFeedforwards.zeros(4));
  private ApplyRobotSpeeds m_pathApplyRobotSpeeds = new ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
  public SwerveRequest calculateFollowRequest(SwerveSetpoint previous, SwerveSample sample) {
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
    var pose = state().Pose;
    var targetSpeeds = sample.getChassisSpeeds();
    // Calculate module forces prior to PID correction
    var targetSpeedsRobotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, getPoseHeading());
    var swerveSetpoint = m_pathPlannerGenerator.generateSetpoint(
      previous,
      targetSpeedsRobotRelative, null, 0.02);
    //
    targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(pose.getRotation().getRadians(),
        sample.heading);
    targetSpeedsRobotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, getPoseHeading());
    m_previousSwerveSetpoint = swerveSetpoint;
    return
        m_pathApplyRobotSpeeds
            .withSpeeds(targetSpeedsRobotRelative)
            .withWheelForceFeedforwardsX(swerveSetpoint.feedforwards().robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(swerveSetpoint.feedforwards().robotRelativeForcesYNewtons())
            ;
  }

  Capture<Pose2d> start = new Capture<Pose2d>(new Pose2d());
  // The goal (populated from poseSupplier at command start)
  Capture<Pose2d> end = new Capture<Pose2d>(new Pose2d());
  // Distance start-end in meters
  Capture<Double> dist = new Capture<Double>(1.0);
  // Unit vector start->end
  Capture<Translation2d> normDirStartToEnd = new Capture<>(Translation2d.kZero);
  TrapezoidProfile.State translationState = new State(0, 0);
  TrapezoidProfile.State rotationState = new State(0, 0);

  // Threshold for "close enough" to avoid microadjustments
  Trigger atPose = atPose(()->end.inner, Units.inchesToMeters(0.5), Units.degreesToRadians(1));
  /**
   * Drives to a pose with motion profiles on translation and rotation.
   * The translation profile starts at dist(start,end) and drives toward 0. This state is then interpolated 
   * between poses.
   * 
   * The rotation profile starts at start.heading and ends at end.heading, just like a profiled continuous heading controller.
   * @param poseSupplier
   * @return
   */
  public Command driveToPoseSupC(Supplier<Pose2d> poseSupplier) {
    var startTime = Timer.getFPGATimestamp();

    var command = runOnce(
        () -> {
          // save the start pose and target pose
          start.inner = getPose();
          var getTargetTime = Timer.getFPGATimestamp();
          end.inner = poseSupplier.get();
          SmartDashboard.putNumber("getTargetTime", Timer.getFPGATimestamp()-getTargetTime);
          var setupTime = Timer.getFPGATimestamp();
          // save the normalized vector from current to target
          normDirStartToEnd.inner = end.inner.getTranslation().minus(start.inner.getTranslation());
          dist.inner = normDirStartToEnd.inner.getNorm();
          normDirStartToEnd.inner = normDirStartToEnd.inner.div(dist.inner + 0.001);
          // initial position: distance from end
          // initial velocity: component of velocity straight towards end (as a negative number)

          translationState.position = dist.inner;
          // Pathing.velocityTowards is negative if approaching the target
          translationState.velocity = 
          MathUtil.clamp(
              Pathing.velocityTowards(
                  start.inner,
                  getFieldRelativeLinearSpeedsMPS(),
                  end.inner.getTranslation()), -driveToPoseConstraints.maxVelocity, 0);
          // Initial state of rotation
          driveToPoseRotationGoal.position = end.inner.getRotation().getRadians();
          
          rotationState.position = start.inner.getRotation().getRadians();
          rotationState.velocity = state().Speeds.omegaRadiansPerSecond;
          SmartDashboard.putNumber("driveToPoseTransInterp", translationState.position);
          // SmartDashboard.putNumber("driveToPoseRotationInterp", rotationState.position);
          // SmartDashboard.putNumber("driveToPoseTransInterpVel", translationState.velocity);
          // SmartDashboard.putNumber("driveToPoseRotationInterpVel", rotationState.velocity);
          SmartDashboard.putNumber("driveToPoseTransDist", dist.inner);
          SmartDashboard.putNumber("setupTime", Timer.getFPGATimestamp()-setupTime);
          // TODO reset previous swerve setpoint
          //m_previousSwerveSetpoint = new SwerveSetpoint(state().Speeds, state().ModuleStates, DriveFeedforwards.zeros(4));

        })
        .andThen(
            run(
                () -> {
                  var setpoint = driveToPoseProfile.calculate(0.02, translationState, driveToPoseGoal);
                  translationState.position = setpoint.position;
                  translationState.velocity = setpoint.velocity;
                  // Rotation continuous input
                  // Get error which is the smallest distance between goal and measurement
                  double errorBound = Math.PI;
                  var measurement = getPoseHeading().getRadians();
                  double goalMinDistance =
                      MathUtil.inputModulus(driveToPoseRotationGoal.position-measurement, -errorBound, errorBound);
                  double setpointMinDistance =
                      MathUtil.inputModulus(rotationState.position - measurement, -errorBound, errorBound);

                  // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
                  // may be outside the input range after this operation, but that's OK because the controller
                  // will still go there and report an error of zero. In other words, the setpoint only needs to
                  // be offset from the measurement by the input range modulus; they don't need to be equal.
                  driveToPoseRotationGoal.position = goalMinDistance + measurement;
                  rotationState.position = setpointMinDistance + measurement;

                  var rotSetpoint = driveToPoseRotationProfile.calculate(
                      0.02, rotationState, driveToPoseRotationGoal);
                  rotationState.position = rotSetpoint.position;
                  rotationState.velocity = rotSetpoint.velocity;
                  SmartDashboard.putNumber("driveToPoseTransInterp", setpoint.position);
                  SmartDashboard.putNumber("driveToPoseRotationInterp", rotSetpoint.position);
                  SmartDashboard.putNumber("driveToPoseTransDist", dist.inner);
                  // SmartDashboard.putNumber("driveToPoseTransInterpVel", translationState.velocity);
                  // SmartDashboard.putNumber("driveToPoseRotationInterpVel", rotationState.velocity);
                  var startPose = start.inner;

                  var interpTrans = end.inner
                      .getTranslation()
                      .interpolate(startPose.getTranslation(), setpoint.position / dist.inner);
                  // var request = calculateFollowRequest(m_previousSwerveSetpoint, RepulsorFieldPlanner.sample(
                  //   interpTrans,
                  //   new Rotation2d(rotationState.position),
                  //   normDirStartToEnd.inner.getX() * -setpoint.velocity,
                  //   normDirStartToEnd.inner.getY() * -setpoint.velocity,
                  //   rotationState.velocity));
                  // if (atPose.getAsBoolean()) {
                  //   this.setControl(xBrake);
                  // } else {
                  //   this.setControl(request);
                  // }

                  if (atPose.getAsBoolean()) {
                    this.setControl(idle);
                  } else {
                    followPath(RepulsorFieldPlanner.sample(
                    interpTrans,
                    new Rotation2d(rotationState.position),
                    normDirStartToEnd.inner.getX() * -setpoint.velocity,
                    normDirStartToEnd.inner.getY() * -setpoint.velocity,
                    rotationState.velocity));
                  }
                }))
;
var endTime = Timer.getFPGATimestamp();
SmartDashboard.putNumber("driveToPoseTime", endTime-startTime);
return command;
  }


  public Command driveToPoseC(Optional<Pose2d> poseOpt) {
    return poseOpt.map((pose) -> driveToPoseSupC(() -> pose)).orElse(Commands.none());
  }

  public Command driveToPoseC(Pose2d poseSup) {
    return driveToPoseSupC(() -> poseSup);
  }

  // END DriveToPose 


  public double toleranceMeters = Units.inchesToMeters(0.5);
  public double toleranceRadians = Units.degreesToRadians(1);

  private boolean withinTolerance(Rotation2d lhs, Rotation2d rhs, double toleranceRadians) {
    if (Math.abs(toleranceRadians) > Math.PI) {
      return true;
    }
    double dot = lhs.getCos() * rhs.getCos() + lhs.getSin() * rhs.getSin();
    // cos(θ) >= cos(tolerance) means |θ| <= tolerance, for tolerance in [-pi, pi],
    // as pre-checked
    // above.
    return dot > Math.cos(toleranceRadians);
  }
  public Trigger atPose(Supplier<Pose2d> poseSup, double toleranceMeters, double toleranceRadians) {
    return new Trigger(
        () -> {
          Pose2d pose = poseSup.get();
          Pose2d currentPose = getPose();
          boolean transValid = currentPose.getTranslation().getDistance(pose.getTranslation()) < toleranceMeters;
          boolean rotValid = withinTolerance(currentPose.getRotation(), pose.getRotation(), toleranceRadians);
          return transValid && rotValid;
        });
  }

  public Trigger atPose(Supplier<Pose2d> poseSup) {
    return atPose(poseSup, toleranceMeters, toleranceRadians);
  }

  public Trigger atPose(Optional<Pose2d> poseOpt) {
    return poseOpt.map(this::atPose).orElse(new Trigger(() -> false));
  }

  public Trigger atPose(Pose2d pose) {
    return atPose(() -> pose);
  }

  public Trigger safeToMoveArm(Supplier<Pose2d> poseSup) {
    return atPose(poseSup, Units.inchesToMeters(24), 2*Math.PI);
  }

  public Trigger safeToReefAlign(Supplier<Pose2d> reefTargetSup) {
    return new Trigger(()->{
      var target = reefTargetSup.get();
      var pose = getPose();
      var distance = target.getTranslation().getDistance(pose.getTranslation());
      var rightSide = pose.relativeTo(target).getX()>0;
      return distance<2 && rightSide;
    });
  }

public Command goToPosition(Supplier<Pose2d> target) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'goToPosition'");
}
}
