// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

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
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.RealArm;
import frc.robot.subsystems.led.LightStripS;
import frc.robot.subsystems.led.OuterStrip.OuterStates;
import frc.robot.subsystems.led.TopStrip.TopStates;
import frc.robot.util.AlertsUtil;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends TimedRobot {
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final OperatorBoard m_operatorBoard =
      Robot.isReal() ? new RealOperatorBoard(1) : new SimOperatorBoard(1);
  private final DriveBaseS m_drivebaseS = TunerConstants.createDrivetrain();
  private final RealArm m_arm = new RealArm();
  private final IntakeS m_hand = new IntakeS();
  private final ClimbWheelsS m_climbWheelsS = new ClimbWheelsS();
  private final ArmBrakeS m_armBrakeS = new ArmBrakeS();
  private final Autos m_autos =
      new Autos(
          m_drivebaseS, m_arm, m_hand, m_operatorBoard, m_armBrakeS, (traj, isStarting) -> {});
  private final SwerveRequest.FieldCentric m_driveRequest =
      new FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
  public FieldCentricFacingAngle m_headingAlignRequest =
      new FieldCentricFacingAngle()
          .withHeadingPID(7, 0, 0)
          .withDriveRequestType(DriveRequestType.Velocity);
  private boolean allHomed = false;

  private final DriverDisplay m_driverDisplay = new DriverDisplay();

  private Mechanism2d VISUALIZER;

  // For logging
  public Pose3d[] components() {
    return RobotVisualizer.getComponents();
  }

  private void setAllHomed(boolean homed) {
    this.allHomed = homed;
  }

  private boolean realRobotIsInWorkshop = true;
  private Trigger inWorkshop =
      new Trigger(() -> RobotBase.isReal() && realRobotIsInWorkshop)
          .and(() -> !DriverStation.isFMSAttached());

  // Coast mode button on pivot A-frame
  private DigitalInput coastButton = new DigitalInput(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    SignalLogger.enableAutoLogging(false);
    LightStripS.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    VISUALIZER = RobotVisualizer.MECH_VISUALIZER;
    Epilogue.bind(this);
    m_driverDisplay
        .setAllHomedSupplier(() -> false)
        .setHasCoralSupplier(m_autos::hasCoral)
        .setBranchSupplier(m_operatorBoard::getBranch)
        .setClimbSupplier(m_autos::selectedClimbNumber)
        .setLevelSupplier(m_operatorBoard::getLevel)
        .setAllHomedSupplier(() -> this.allHomed);
    AlertsUtil.bind(
        new Alert("Driver Xbox Disconnect", AlertType.kError),
        () -> !m_driverController.isConnected());
    var algaeAlignButton = m_driverController.x();
    m_drivebaseS.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivebaseS.applyRequest(
            () -> {
              final var maxSpeed = 4.2;
              var xSpeed = -m_driverController.getLeftY() * maxSpeed;
              var ySpeed = -m_driverController.getLeftX() * maxSpeed;
              var rotationSpeed = -m_driverController.getRightX() * 2 * Math.PI;

              // Ignore joysticks in auto, just in case they're non-centered (this happened once)
              if (DriverStation.isAutonomous()) {
                return m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
              }
              // Heading align closest pivot or battery side to closest reef face for algae de-reef
              if (algaeAlignButton.getAsBoolean()) {
                return m_headingAlignRequest
                    .withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withTargetDirection(m_autos.closerAlgaeAlignHeadingAllianceRelative());
              }
              return m_driveRequest
                  .withVelocityX(xSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(ySpeed) // Drive left with negative X (left)
                  .withRotationalRate(rotationSpeed);
            } // Drive counterclockwise with negative X (left)
            ));

    RobotVisualizer.setupVisualizer();

    RobotVisualizer.addArmPivot(m_arm.ARM);
    SmartDashboard.putData("visualizer", VISUALIZER);
    SmartDashboard.putData("autoChooser", m_autos.m_autoChooser);

    configureDriverController();
    configureOperatorController();
    new Trigger(m_hand.m_coralSensor::hasCoral).onFalse(RobotVisualizer.markCoralLeavesHand(m_drivebaseS::getPose, m_hand::getCoralInlineOffset));
    DriverStation.silenceJoystickConnectionWarning(true);
    RobotModeTriggers.autonomous().whileTrue(m_autos.m_autoChooser.selectedCommandScheduler());
  }

  private void configureOperatorController() {
    /*
     * Prepare to approach the cage. 
     */
    m_operatorBoard
        .left()
        .onTrue(m_arm.goToPosition(Arm.Positions.PRE_CLIMB))
        .onTrue(m_climbWheelsS.in());

    /*
     * Rotate pivot down at fast speed
     * When the pivot is below 70 degrees, tuck the intake in.
     * When the pivot is below 40 degrees, extend the elevator for CoG manipulation.
     * When the pivot is below 10 degrees, give a roughly static pivot voltage to stop without much oscillation.
     */
    m_operatorBoard
        .center()
        .onTrue(m_climbWheelsS.in())
        .whileTrue(
            parallel(
                LightStripS.top.stateC(() -> TopStates.Climbing),
                LightStripS.outer.stateC(() -> OuterStates.Climbing),
                parallel(
                    m_arm.mainPivotS.voltage(
                        () ->
                            (m_arm.mainPivotS.getAngleRadians() < Units.degreesToRadians(10))
                                ? -0.5
                                : -5),
                    Commands.waitUntil(
                            () ->
                                m_arm.mainPivotS.getAngleRotations() < Units.degreesToRotations(70))
                        .andThen(m_arm.wristS.goTo(() -> Units.degreesToRadians(90 + 35))),
                    Commands.waitUntil(
                            () ->
                                m_arm.mainPivotS.getAngleRotations() < Units.degreesToRotations(40))
                        .andThen(m_arm.elevatorS.goToLength(() -> 1.05)))));
    /*
     * Engage brake and stop wheels.
     */
    m_operatorBoard
        .right()
        .onTrue(m_armBrakeS.brake())
        .onFalse(m_armBrakeS.release())//Often not actually possible, as brake is too tightly engaged.
        .onTrue(m_climbWheelsS.stop());
  }

  public void configureDriverController() {

    /*
     * Coral intake, either wall or ground based on operator toggle.
     * For ground position, pressing the button causes a short shutoff of the intake if already running.
     * This was needed to push past occasional coral stalls.
     */
    m_driverController
        .a()
        .onTrue(
            Commands.either(
                m_autos.autoCoralIntake(),
                sequence(
                    m_hand
                        .voltage(2)
                        .withTimeout(0.1)
                        .onlyIf(() -> m_hand.getVoltage() > 0.02)
                        .asProxy(),
                    m_autos.autoCoralGroundIntake().asProxy()),
                m_operatorBoard.toggle()));

    /*
     * Processor scoring position (also works for lollipop algae intake).
     * Since the wrist is at its hard stop, this command also homes the wrist (added for Daly playoffs)
     */
    m_driverController.back().onTrue(m_hand.inAlgae()).onTrue(m_arm.processorWithHome());

    /*
     * Auto align to closest net scoring line
     *  Takes over X driving and rotation while allowing joystick control along the line.
     */
    m_driverController
        .start()
        .whileTrue(m_autos.alignToBarge(() -> -m_driverController.getLeftX() * 4));
    /*
     * Auto net algae launch. Assumes robot is already in drivetrain position. Does not assume arm is already in stow.
     */
    m_driverController.y().onTrue(m_autos.bargeUpAndOutVoltage());
    /*
     * Algae de-reef.
     * According to the pose at button press, move arm to the right algae position (high or low, pivot or battery side)
     * and start intake.
     * NOTE: heading align to reef face according to current pose is handled in the drive default command.
     */
    m_driverController.x().onTrue(m_autos.armToClosestAlgae()).onTrue(m_hand.inAlgae());

    /*
     * Ground algae position and start intake.
     */
    m_driverController
        .b()
        .onTrue(m_arm.goToPosition(Arm.Positions.GROUND_ALGAE))
        .onTrue(m_hand.inAlgae());
    /*
     * Go to the mostly-upright stow position.
     */
    m_driverController.leftBumper().onTrue(m_arm.goToPosition(Arm.Positions.STOW));


    /**
     * Score coral and stow. To prevent driving coral into the elevator, do L1 outtake if the hand is too close to the hard stop.
     * This logic handles the actual scoring when in L1 position too.
     * After the score, schedule the stow command for the last scoring option.
     */
    m_driverController
        .rightBumper()
        .onTrue(
            either(
                    m_hand.voltage(() -> ReefScoringOption.L1.outtakeVoltage).withTimeout(0.5), // spit out if not safe to
                    m_hand
                        .voltage(() -> m_autos.lastScoringOption.inner.outtakeVoltage)
                        .withTimeout(0.5),
                    () -> m_arm.wristS.getAngleRadians() < Units.degreesToRadians(20))
                .andThen(new ScheduleCommand(m_autos.stowAfterCoral(m_autos.lastScoringOption))));

    /*
     * Algae score. 
     * Score/drop algae and stow if at barge position. Elevator length check prevents stowing after processor outtake.
     * TODO: Remove the stow altogether: it's not used with current auto barge scoring.
     */
    m_driverController
        .leftTrigger()
        .onTrue(
            parallel(m_hand.outAlgaeSlow().withTimeout(0.5))
                .andThen(
                    new ScheduleCommand(m_arm.goToPosition(Arm.Positions.STOW))
                        .onlyIf(
                            () ->
                                m_arm.getPosition().elevatorMeters()
                                    > Arm.Positions.L3_PIV.elevatorMeters())));
    /*
     * Activate auto align and arm position for selected coral scoring option.
     */ 
    m_driverController.rightTrigger().whileTrue(m_autos.autoScoreMap());

    // HOME arm brake in pre-match position (not fully in, but not loose)
    m_driverController
        .leftStick()
        .and(m_driverController.rightStick())
        .and(RobotModeTriggers.disabled())
        .onTrue(m_armBrakeS.home())
        .onTrue(LightStripS.top.stateC(() -> TopStates.Intaked).withTimeout(0.5));

    m_driverController.povCenter().negate().whileTrue(driveIntakeRelativePOV());

    /**
     * Mark robot in starting configuration:
     *  * home wrist
     *  * home elevator
     *  Brake homed separately.
     */
    m_driverController
    .start()
    .and(RobotModeTriggers.disabled())
    .onTrue(
        parallel(
                m_arm.elevatorS.home(),
                m_arm.wristS.home(),
                Commands.runOnce(() -> this.setAllHomed(true)).ignoringDisable(true))
            .ignoringDisable(true));
    
    // Coast mode on wrist and pivot while controller button or on-robot button are held
    m_driverController
    .back()
    .or(() -> !coastButton.get())
    .and(RobotModeTriggers.disabled())
    .whileTrue(
        parallel(
            m_arm.mainPivotS.coast(),
            m_arm.wristS.coast(),
            LightStripS.top.stateC(() -> TopStates.CoastMode)));
  }


  // Robot-centric drive in the D-Pad direction, with joystick rotation speed control.
  private RobotCentric m_robotCentricRequest =
      new RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

  public Command driveIntakeRelativePOV() {
    return m_drivebaseS.applyRequest(
        () -> {
          double pov = Units.degreesToRadians(-m_driverController.getHID().getPOV());
          double adjustSpeed = Units.feetToMeters(3);// per second
          return m_robotCentricRequest
              .withVelocityX(Math.cos(pov) * adjustSpeed)
              .withVelocityY(Math.sin(pov) * adjustSpeed)
              .withRotationalRate(-m_driverController.getRightX() * 2 * Math.PI);
        });
  }

  @NotLogged private double lastTimestamp = Timer.getFPGATimestamp();
  @NotLogged private int lastRobotHeartbeat = 0;

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Decode selections from operator board
    m_operatorBoard.poll();
    
    m_driverDisplay.update();
    // Update cached arm position and visualization
    m_arm.update();
    RobotVisualizer.setArmPosition(m_arm.getPosition());
    Epilogue.talonFXLogger.refreshAll();
    CommandScheduler.getInstance().run();

    DriverStation.getAlliance()
        .ifPresent(
            alliance -> {
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
    // Update light strips
    LightStripS.periodic();

    var loopTime = Timer.getFPGATimestamp() - lastTimestamp;
    SmartDashboard.putNumber("loopTime", loopTime);
    lastTimestamp = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("robotHeartbeat", lastRobotHeartbeat++);
  }

  public Pose3d getCoralPose() {
    return RobotVisualizer.getCoralPose(m_drivebaseS.getPose(), m_hand.getCoralInlineOffset(), m_autos.hasCoral());
  }

  public Pose3d getAlgaePose() {
    return RobotVisualizer.getAlgaePose(m_drivebaseS.getPose());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
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


  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_drivebaseS.stopBrakeMode().ignoringDisable(true).until(() -> true).schedule();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
