// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.ArrayList;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.operator.OperatorBoard;
import frc.operator.RealOperatorBoard;
import frc.operator.SimOperatorBoard;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmBrakeS;
import frc.robot.subsystems.ClimbWheelsS;
// import frc.robot.logging.TalonFXLogger;
import frc.robot.subsystems.DriveBaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.IntakeS.HandConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.RealArm;
import frc.robot.subsystems.led.LightStripS;
import frc.robot.subsystems.led.OuterStrip.OuterStates;
import frc.robot.subsystems.led.TopStrip.TopStates;
import frc.robot.util.AlertsUtil;
import frc.robot.util.Capture;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends TimedRobot {
  // public PDData pdh = PDData.create(1, ModuleType.kRev);
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final OperatorBoard m_operatorBoard = Robot.isReal() ? new RealOperatorBoard(1) : new SimOperatorBoard(1);
  private final DriveBaseS m_drivebaseS = TunerConstants.createDrivetrain();
  private final RealArm m_arm = new RealArm();
  private final IntakeS m_hand = new IntakeS();
  // private final ClimbHookS m_climbHookS = new ClimbHookS();
  private final ClimbWheelsS m_climbWheelsS = new ClimbWheelsS();
  private final ArmBrakeS m_armBrakeS = new ArmBrakeS();
  private final Autos m_autos = new Autos(m_drivebaseS, m_arm, m_hand, m_operatorBoard, m_armBrakeS,
      (traj, isStarting) -> {
      });
  private final SwerveRequest.FieldCentric m_driveRequest = new FieldCentric()
      .withDriveRequestType(DriveRequestType.Velocity);
  public FieldCentricFacingAngle m_headingAlignRequest = new FieldCentricFacingAngle().withHeadingPID(7, 0, 0)
      .withDriveRequestType(DriveRequestType.Velocity);
  private boolean allHomed = false;
  // private final CommandOperatorKeypad m_keypad = new CommandOperatorKeypad(5);
  // private final DrivetrainSysId m_driveId = new DrivetrainSysId(m_drivebaseS);
  private final DriverDisplay m_driverDisplay = new DriverDisplay();

  public static final CANBus m_notSwerveBus = new CANBus("NotSwerve");

  private Mechanism2d VISUALIZER;

  public Pose3d[] components() {
    return RobotVisualizer.getComponents();
  }

  private void setAllHomed(boolean homed) {
    this.allHomed = homed;
  }

  private boolean realRobotIsInWorkshop = true;
  private Trigger inWorkshop = new Trigger(() -> RobotBase.isReal() && realRobotIsInWorkshop)
      .and(() -> !DriverStation.isFMSAttached());

  private DigitalInput coastButton = new DigitalInput(0);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    SignalLogger.enableAutoLogging(false);
    LightStripS.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    VISUALIZER = RobotVisualizer.MECH_VISUALIZER;
    Epilogue.bind(this);
    m_driverDisplay.setAllHomedSupplier(() -> false)
        .setHasCoralSupplier(m_autos::hasCoral)
        .setBranchSupplier(m_operatorBoard::getBranch)
        .setClimbSupplier(m_autos::selectedClimbNumber)
        .setLevelSupplier(m_operatorBoard::getLevel)
        .setAllHomedSupplier(() -> this.allHomed);
    AlertsUtil.bind(
        new Alert("Driver Xbox Disconnect", AlertType.kError),
        () -> !m_driverController.isConnected());
    var intakeAlignButton = m_driverController.a();
    var algaeAlignButton = m_driverController.x();
    m_drivebaseS.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivebaseS.applyRequest(
            () -> {
              var xSpeed = -m_driverController.getLeftY() * 4.2;
              var ySpeed = -m_driverController.getLeftX() * 4.2;
              var rotationSpeed = -m_driverController.getRightX() * 2 * Math.PI;

              if (DriverStation.isAutonomous()) {
                return m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
              }
              // if (intakeAlignButton.getAsBoolean()) {
              // return
              // m_headingAlignRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withTargetDirection(
              // m_autos.intakeHeadingAllianceRelative()
              // );
              // }
              if (algaeAlignButton.getAsBoolean()) {
                return m_headingAlignRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withTargetDirection(
                    m_autos.closerAlgaeAlignHeadingAllianceRelative());
              }
              return m_driveRequest
                  .withVelocityX(
                      xSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(
                      ySpeed) // Drive left with negative X (left)
                  .withRotationalRate(
                      rotationSpeed);
            } // Drive counterclockwise with negative X (left)
        ));

    RobotVisualizer.setupVisualizer();

    RobotVisualizer.addArmPivot(m_arm.ARM);
    // RobotVisualizer.addAlgaeIntake(m_algaePivotS.ALGAE_PIVOT);
    SmartDashboard.putData("visualizer", VISUALIZER);
    SmartDashboard.putData("autoChooser", m_autos.m_autoChooser);

    configureDriverController();
    // Coast mode when disabled
    m_driverController.back().or(() -> !coastButton.get()).and(RobotModeTriggers.disabled()).whileTrue(
        parallel(m_arm.mainPivotS.coast(), m_arm.wristS.coast(), LightStripS.top.stateC(() -> TopStates.CoastMode)));
    // .whileTrue(m_climbHookS.coast());
    // Home wrist when disabled
    m_driverController.start().and(RobotModeTriggers.disabled())
        .onTrue(parallel(
            m_arm.elevatorS.home(),
            m_arm.wristS.home(),
            Commands.runOnce(() -> this.setAllHomed(true)).ignoringDisable(true)).ignoringDisable(true));
    m_driverController.povCenter().negate().whileTrue(driveIntakeRelativePOV());
    configureOperatorController();
    new Trigger(m_hand.m_coralSensor::hasCoral).onFalse(
        Commands.runOnce(() -> {
          lastCoralPose.inner = getCoralPoseIfInHand();
        }).ignoringDisable(true));
    DriverStation.silenceJoystickConnectionWarning(true);
    RobotModeTriggers.autonomous().whileTrue(m_autos.m_autoChooser.selectedCommandScheduler());
  }

  private void configureOperatorController() {
    m_operatorBoard.left().onTrue(
        m_arm.goToPosition(Arm.Positions.PRE_CLIMB))
        .onTrue(m_climbWheelsS.in());
    // .onTrue(
    // m_climbHookS.release().withTimeout(5)
    // );
    m_operatorBoard.center().onTrue(m_climbWheelsS.in())// .onTrue(m_climbHookS.clamp())
        .whileTrue(parallel(LightStripS.top.stateC(() -> TopStates.Climbing),
            LightStripS.outer.stateC(() -> OuterStates.Climbing),
            parallel(
                m_arm.mainPivotS
                    .voltage(() -> (m_arm.mainPivotS.getAngleRadians() < Units.degreesToRadians(10)) ? -0.5 : -5),
                Commands.waitUntil(() -> m_arm.mainPivotS.getAngleRotations() < Units.degreesToRotations(70))
                    .andThen(
                        m_arm.wristS.goTo(() -> Units.degreesToRadians(90 + 35))),
                Commands.waitUntil(() -> m_arm.mainPivotS.getAngleRotations() < Units.degreesToRotations(40))
                    .andThen(
                        m_arm.elevatorS.goToLength(() -> 1.05))

            )));
    m_operatorBoard.right().onTrue(m_armBrakeS.brake()).onFalse(m_armBrakeS.release())
        .onTrue(m_climbWheelsS.stop());
  }

  public void configureDriverController() {

    // align to closest coral station (or left station if in workshop)
    m_driverController.a().onTrue(Commands.either(
        m_autos.autoCoralIntake(),
        Commands.either(
            // TODO: add actual ground l1 intake functioanlity!!!
            m_autos.coralIntakeL1().asProxy(),
            sequence(
                m_hand.voltage(2).withTimeout(0.1).onlyIf(() -> m_hand.getVoltage() > 0.02).asProxy(),
                m_autos.autoCoralGroundIntake().asProxy()),
            () -> m_operatorBoard.getLevel() == 0
        ),
        m_operatorBoard.toggle()));

    // go to processor position
    m_driverController.back()
        .onTrue(m_hand.inAlgae())
        .onTrue(sequence(
            m_arm.processorWithHome()));

    // Align to barge
    m_driverController.start()// .and(inWorkshop.negate())
        .whileTrue(
            parallel(
                m_autos.alignToBarge(() -> -m_driverController.getLeftX() * 4)));

    m_driverController.y()
    .onTrue(m_autos.bargeUpAndOutVoltage());

    // .onTrue(m_hand.inAlgae());
    // Intake algae from reef (autoalign, move arm to position, intake and stow)
    m_driverController.x()
        .onTrue(m_autos.armToClosestAlgae())
        .onTrue(m_hand.inAlgae());

    // Stow
    m_driverController.b().onTrue(m_arm.goToPosition(Arm.Positions.GROUND_ALGAE)).onTrue(m_hand.inAlgae());
    //m_driverController.leftBumper().onTrue(m_arm.goToPosition(Arm.Positions.STOW));
    // m_arm.goToPosition(Arm.Positions.STOW));
    // Score coral and stow
    boolean coralPivotSide = false;
    m_driverController.rightBumper().onTrue(
        either(m_hand.voltage(() -> -4).withTimeout(0.5), // spit out if not safe to

            m_hand.voltage(() -> m_autos.lastScoringOption.inner.outtakeVoltage).withTimeout(0.5),

            () -> m_arm.wristS.getAngleRadians() < Units.degreesToRadians(20))
            .andThen(new ScheduleCommand(m_autos.stowAfterCoral(m_autos.lastScoringOption))));

    // Score algae and stow if at barge position
    m_driverController.leftTrigger().onTrue(parallel(
    m_hand.outAlgaeSlow().withTimeout(0.5)).andThen(
    new ScheduleCommand(m_arm.goToPosition(Arm.Positions.STOW))
    .onlyIf(() -> m_arm.getPosition().elevatorMeters() >
    Arm.Positions.L3.elevatorMeters())));

    //m_driverController.y().onTrue(m_hand.m_coralSensor.setHasCoralC(!m_hand.m_coralSensor.hasCoral()));

    // Auto align to operator selected position on reef for coral scoring
    m_driverController.rightTrigger().whileTrue(m_autos.autoScoreMap());

    /*
     * m_driverController.leftStick().whileTrue(Commands.none());
     * m_driverController.rightStick().whileTrue(Commands.none());
     */

    // HOME arm brake
    m_driverController.leftStick().and(m_driverController.rightStick()).and(RobotModeTriggers.disabled())
        .onTrue(m_armBrakeS.home())
        .onTrue(LightStripS.top.stateC(() -> TopStates.Intaked).withTimeout(0.5));

    m_driverController.povCenter().negate().whileTrue(driveIntakeRelativePOV());

  }

  private RobotCentric m_robotCentricRequest = new RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

  public Command driveIntakeRelativePOV() {
    return m_drivebaseS.applyRequest(() -> {
      double pov = Units.degreesToRadians(-m_driverController.getHID().getPOV());
      double adjustSpeed = Units.feetToMeters(3); // m/s
      return m_robotCentricRequest.withVelocityX(
          Math.cos(pov) * adjustSpeed).withVelocityY(
              Math.sin(pov) * adjustSpeed)
          .withRotationalRate(
              -m_driverController.getRightX() * 2
                  * Math.PI);
    });
  }

  ArrayList<Translation2d> toGoal = new ArrayList<>();
  Pose3d emptyPose = Pose3d.kZero;
  @NotLogged
  private double lastTimestamp = Timer.getFPGATimestamp();
  @NotLogged
  private int lastRobotHeartbeat = 0;

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_operatorBoard.poll();
    m_driverDisplay.update();

    m_arm.update();
    RobotVisualizer.setArmPosition(m_arm.getPosition());
    Epilogue.talonFXLogger.refreshAll();
    // pdh.update();
    // if (m_operatorBoard.getToggle()) {
    // LightStripS.top.requestState(TopStates.ReadyToIntake);
    // }
    CommandScheduler.getInstance().run();
    LightStripS.periodic();

    DriverStation.getAlliance().ifPresent(alliance -> {
      if (alliance == Alliance.Red) {
        LightStripS.top.requestState(TopStates.RedAlliance);
        LightStripS.outer.requestState(OuterStates.RedAlliance);
      } else {
        LightStripS.top.requestState(TopStates.BlueAlliance);
        LightStripS.outer.requestState(OuterStates.BlueAlliance);
      }
    });
    if (m_autos.drivetrainSafeToAlignTrig.getAsBoolean()) {
      LightStripS.outer.requestSafeToAlign();
    }

    var loopTime = Timer.getFPGATimestamp() - lastTimestamp;
    SmartDashboard.putNumber("loopTime", loopTime);
    lastTimestamp = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("robotHeartbeat", lastRobotHeartbeat++);

  }

  Capture<Pose3d> lastCoralPose = new Capture<Pose3d>(Pose3d.kZero);

  private Pose3d getCoralPoseIfInHand() {
    return new Pose3d(m_drivebaseS.getPose()).plus(new Transform3d(
        RobotVisualizer.getComponents()[3].getTranslation(),
        RobotVisualizer.getComponents()[3].getRotation())).plus(
            new Transform3d(
                HandConstants.CORAL_LENGTH_METERS / 2.0 - Units.inchesToMeters(0.9) - m_hand.getCoralInlineOffset(),
                0.0, -Units.inchesToMeters(9.978 - 0.25), Rotation3d.kZero));
  }

  public Pose3d getCoralPose() {
    if (m_autos.hasCoral()) {
      return getCoralPoseIfInHand();
    } else {
      return lastCoralPose.inner;
    }
  }

  public Pose3d getAlgaePose() {
    return new Pose3d(m_drivebaseS.getPose()).plus(new Transform3d(
        RobotVisualizer.getComponents()[3].getTranslation(),
        RobotVisualizer.getComponents()[3].getRotation())).plus(
            new Transform3d(
                0.42, 0, -0.02, Rotation3d.kZero));
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    if (RobotBase.isSimulation()) {
      Commands.waitSeconds(15)
          .andThen(
              () -> {
                DriverStationSim.setEnabled(false);
                DriverStationSim.notifyNewData();
              })
          .onlyWhile(DriverStation::isAutonomousEnabled)
          .schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // switch (m_autos.selectedReefPOI()) {
    // case A:
    // case B:
    // LightStripS.outer.requestState(OuterStates.AB);
    // break;
    // case C:
    // case D:
    // LightStripS.outer.requestState(OuterStates.CD);
    // break;
    // case E:
    // case F:
    // LightStripS.outer.requestState(OuterStates.EF);
    // break;
    // case G:
    // case H:
    // LightStripS.outer.requestState(OuterStates.GH);
    // break;
    // case I:
    // case J:
    // LightStripS.outer.requestState(OuterStates.IJ);
    // break;
    // case K:
    // case L:
    // LightStripS.outer.requestState(OuterStates.KL);
    // break;
    // default:
    // LightStripS.outer.requestState(OuterStates.Default);
    // }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_drivebaseS.stopBrakeMode().ignoringDisable(true).until(() -> true).schedule();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
