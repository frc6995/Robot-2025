package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.Idle;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.util.RepulsorFieldPlanner;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.logging.Module;
import frc.robot.logging.TalonFXPDHChannel;
import static frc.robot.logging.PowerDistributionSim.Channel.*;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
@Logged
public class DriveBaseS extends TunerSwerveDrivetrain implements Subsystem {
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
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final PIDController m_pathXController = new PIDController(10, 0, 0);
    private final PIDController m_pathYController = new PIDController(10, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(7, 0, 0);

    public RepulsorFieldPlanner m_repulsor = new RepulsorFieldPlanner();
    // For logging
    public Module fl;
    public Module fr;
    public Module bl;
    public Module br;
    private Module makeModule(int idx) {return new Module(this.getModule(idx).getSteerMotor(), this.getModule(idx).getDriveMotor());}
    private void setupModuleLoggers() {
        fl  = makeModule(0);
        fr  = makeModule(1);
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

    /** Re-expose the state as a method of the subclass so Epilogue finds it. */
    public SwerveDriveState state() {
        return getState();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     */
    public DriveBaseS(
        SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, 250, modules);
        setupModuleLoggers();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Pose2d targetPose() {
        return new Pose2d(m_pathXController.getSetpoint(), m_pathYController.getSetpoint(), Rotation2d.fromRadians(m_pathThetaController.getSetpoint()));
    }
    public Command repulsorCommand(Supplier<Pose2d> target) {
        return run(()->{
            m_repulsor.setGoal(target.get().getTranslation());
            followPath(m_repulsor.getCmd(state().Pose, state().Speeds, 4, true, target.get().getRotation()));
        });
    }
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public DriveBaseS(double OdometryUpdateFrequency) {
        super(TunerConstants.DrivetrainConstants, OdometryUpdateFrequency, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
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
     * @param pose Current pose of the robot
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
        var pose = state().Pose;
        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        setControl(
            m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                // .withWheelForceFeedforwardsX(sample.moduleForcesX())
                // .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }
    SwerveRequest.ApplyRobotSpeeds idle = new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds());
    public Command stop () {
        return applyRequest(()->idle);
    }
    private final SwerveSample[] emptyTrajectory = new SwerveSample[0];
    public SwerveSample[] currentTrajectory = emptyTrajectory;
    public void logTrajectory(Trajectory<SwerveSample> traj, boolean isStarting) {
        currentTrajectory = isStarting ? traj.samples().toArray(SwerveSample[]::new) : emptyTrajectory;
    }
    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void updateSimCurrents() {

    }
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

          public Command wheelCharacterization() {
    var rotateRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withRotationalRate(0.4);
    var directions = new double[4];
    var initialWheelPositions = new Distance[4];
    var initialYaw = Rotation.mutable(0);
    return Commands.sequence(
        // Start with a bit of rotation to make sure the wheels are in position:
        applyRequest(() -> rotateRequest).withTimeout(2),
        // Record initial data:
        runOnce(
            () -> {
              for (int i = 0; i < 4; i++) {
                var module = getModule(i);
                directions[i] = Math.copySign(1, module.getCurrentState().speedMetersPerSecond);
                initialWheelPositions[i] = Meters.of(module.getPosition(false).distanceMeters);
              }
              initialYaw.mut_replace(getPigeon2().getYaw().getValue());
            }),
        Commands.parallel(
            applyRequest(() -> rotateRequest),
            Commands.run(
                () -> {
                  // Find the difference in yaw since start
                  var currentYaw = getPigeon2().getYaw().getValue();
                  var gyroYawDifference = currentYaw.minus(initialYaw);
                  SmartDashboard.putNumber(
                      "Wheel characterization gyro difference", gyroYawDifference.in(Radians));

                  // Find how much the wheels have moved
                  var avgWheelMovement = Meters.mutable(0);
                  for (int i = 0; i < 4; i++) {
                    var module = getModule(i);
                    var wheelMovement =
                        (Meters.of(module.getPosition(false).distanceMeters)
                                .minus(initialWheelPositions[i]))
                            .times(directions[i]);
                    avgWheelMovement.mut_plus(wheelMovement);
                  }
                  avgWheelMovement.mut_divide(4);
                  SmartDashboard.putNumber(
                      "Wheel characterization wheel movement", avgWheelMovement.in(Meters));
                  // Find wheel circumference
                  var currentWheelCircumference = TunerConstants.kWheelRadius.times(2 * Math.PI);
                  // Based on wheel circumference, convert wheel movement in meters to rotations
                  var avgWheelMovementAngle =
                      avgWheelMovement.div(currentWheelCircumference).times(Rotation.one());
                  SmartDashboard.putNumber(
                      "Wheel characterization wheel movement radians",
                      avgWheelMovementAngle.in(Radians));

                  // Find the drive base radius of the wheels
                  var drivebaseRadius = Meters.of(getModuleLocations()[0].getNorm());
                  // Find the circumference from this radius
                  var drivebaseCircumference = drivebaseRadius.times(2 * Math.PI);
                  // Find the arc length that was actually traveled by each wheel based on gyro
                  var arcLength =
                      drivebaseCircumference.times(gyroYawDifference.div(Rotation.one()));
                  SmartDashboard.putNumber(
                      "Wheel characterization arc traveled", arcLength.in(Meters));

                  // Find what the wheel circumference should be based on the arc length
                  var actualWheelCircumference =
                      arcLength.div(avgWheelMovementAngle.div(Rotation.one()));
                  // Find what the wheel radius should be
                  var calculatedRadius = actualWheelCircumference.div(2 * Math.PI);
                  // Put to dashboard
                  SmartDashboard.putNumber(
                      "Wheel characterization CALCULATED RADIUS", calculatedRadius.in(Inches));
                })));
            }

 
}