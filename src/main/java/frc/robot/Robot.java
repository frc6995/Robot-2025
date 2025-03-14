// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.defer;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.ArrayList;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
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
import frc.robot.subsystems.ClimbHookS;
// import frc.robot.logging.TalonFXLogger;
import frc.robot.subsystems.DriveBaseS;
import frc.robot.subsystems.RealHandS;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.RealArm;
import frc.robot.subsystems.led.LightStripS;
import frc.robot.subsystems.led.OuterStrip.OuterStates;
import frc.robot.subsystems.led.TopStrip.TopStates;
import frc.robot.util.AlertsUtil;

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
  private final RealHandS m_hand = new RealHandS();
  private final ClimbHookS m_climbHookS = new ClimbHookS();
  //private final ClimbWheelsS m_climbWheelsS = new ClimbWheelsS();
  private final ArmBrakeS m_armBrakeS = new ArmBrakeS();
  private final Autos m_autos = new Autos(m_drivebaseS, m_arm, m_hand, m_operatorBoard, m_armBrakeS,
      (traj, isStarting) -> {
      });
  private final SwerveRequest.FieldCentric m_driveRequest = new FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
  private boolean allHomed = false;
  // private final CommandOperatorKeypad m_keypad = new CommandOperatorKeypad(5);
  // private final DrivetrainSysId m_driveId = new DrivetrainSysId(m_drivebaseS);
  private final DriverDisplay m_driverDisplay = new DriverDisplay();
  
  private Mechanism2d VISUALIZER;

  public Pose3d[] components() {
    return RobotVisualizer.getComponents();
  }

  private void setAllHomed(boolean homed) {
    this.allHomed = homed;
  }

  private boolean realRobotIsInWorkshop = true;
  private Trigger inWorkshop = 
    new Trigger(()->RobotBase.isReal() && realRobotIsInWorkshop)
    .and(()->!DriverStation.isFMSAttached());

  private DigitalInput coastButton = new DigitalInput(0);
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    LightStripS.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    VISUALIZER = RobotVisualizer.MECH_VISUALIZER;
    Epilogue.bind(this);
    m_driverDisplay.setAllHomedSupplier(() -> false)
        .setHasCoralSupplier(m_autos::hasCoral)
        .setBranchSupplier(m_operatorBoard::getBranch)
        .setClimbSupplier(m_autos::selectedClimbNumber)
        .setLevelSupplier(m_operatorBoard::getLevel)
        .setAllHomedSupplier(()->this.allHomed);
    AlertsUtil.bind(
        new Alert("Driver Xbox Disconnect", AlertType.kError),
        () -> !m_driverController.isConnected());
    m_drivebaseS.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivebaseS.applyRequest(
            () -> DriverStation.isAutonomous()
                ? m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
                : m_driveRequest
                    .withVelocityX(
                        -m_driverController.getLeftY()
                            * 5) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -m_driverController.getLeftX() * 5) // Drive left with negative X (left)
                    .withRotationalRate(
                        -m_driverController.getRightX()
                            * 2
                            * Math.PI) // Drive counterclockwise with negative X (left)
        ));

    RobotVisualizer.setupVisualizer();

    RobotVisualizer.addArmPivot(m_arm.ARM);
    // RobotVisualizer.addAlgaeIntake(m_algaePivotS.ALGAE_PIVOT);
    SmartDashboard.putData("visualizer", VISUALIZER);
    SmartDashboard.putData("autoChooser", m_autos.m_autoChooser);

    configureDriverController();
// Coast mode when disabled
    m_driverController.back().or(()->!coastButton.get()).and(RobotModeTriggers.disabled()).whileTrue(
      parallel(m_arm.mainPivotS.coast(), LightStripS.top.stateC(()->TopStates.CoastMode))
    )
    .whileTrue(m_climbHookS.coast());
    m_driverController.back().onTrue(m_climbHookS.release().withTimeout(5));
    //Home wrist when disabled
    m_driverController.start().and(RobotModeTriggers.disabled())
        .onTrue(parallel(
          m_arm.elevatorS.home(),
          m_arm.wristS.home(),
          runOnce(()->this.setAllHomed(true)).ignoringDisable(true)).ignoringDisable(true));
        m_driverController.povCenter().negate().whileTrue(driveIntakeRelativePOV());
    configureOperatorController();
    DriverStation.silenceJoystickConnectionWarning(true);
    RobotModeTriggers.autonomous().whileTrue(m_autos.m_autoChooser.selectedCommandScheduler());
  }

  private void configureOperatorController() {
    m_operatorBoard.left().onTrue(
      m_arm.goToPosition(Arm.Positions.PRE_CLIMB)).onTrue(
        m_climbHookS.release().withTimeout(5)
    );
    m_operatorBoard.center().onTrue(m_climbHookS.clamp())
    .whileTrue(waitSeconds(2).andThen(
      parallel(
        m_arm.mainPivotS.voltage(()->
          (m_arm.mainPivotS.getAngleRadians() < Units.degreesToRadians(30)) ? 0 : -2),
        waitUntil(()->m_arm.mainPivotS.getAngleRotations() < Units.degreesToRotations(60))
          .andThen(
            m_arm.wristS.goTo(()->0.0)
          )))
    );
    m_operatorBoard.right().onTrue(m_armBrakeS.brake()).onFalse(m_armBrakeS.release());
    //.onTrue(m_climbWheelsS.stop());
  }
  public void configureDriverController() {
    
    // TODO: assign buttons to functions specified in comments

    // align to closest coral station (or left station if in workshop)
    m_driverController.a().whileTrue(
        Commands.defer(() -> m_autos.autoCoralIntake(inWorkshop.getAsBoolean() ? POI.SL3 : m_autos.closestIntake()), Set.of(m_drivebaseS)));
    // Drive and autoalign to processor
    // If NOT in workshop, drive to processor.
    m_driverController.start()
        .onTrue(m_hand.inAlgae())
        .onTrue(sequence(
            m_arm.goToPosition(Arm.Positions.SCORE_PROCESSOR)
        ))
        .and(inWorkshop.negate())
        .whileTrue(m_drivebaseS.driveToPoseSupC(POI.PROC::flippedPose));

    // Align and score in barge; stow
    m_driverController.y().and(inWorkshop.negate())
        .onTrue(m_arm.goToPosition(Arm.Positions.SCORE_BARGE.premove()))
        .onTrue(m_hand.inAlgae())
        .whileTrue(
            parallel(
              m_autos.alignToBarge(() -> -m_driverController.getLeftX() * 4),
              waitUntil(m_autos::atBargeLine).andThen(
                m_autos.bargeUpAndOut()
              )
            )
            );
      m_driverController.y().and(inWorkshop)
      .onTrue(m_autos.bargeUpAndOut());
      //.onTrue(m_hand.inAlgae());
    // Intake algae from reef (autoalign, move arm to position, intake and stow)
    m_driverController.x()
    // .whileTrue(
    //     defer(() -> m_drivebaseS.driveToPoseSupC(m_autos.closestSide().algae::flippedPose), Set.of(m_drivebaseS))

    // )
    .whileTrue(
        defer(() -> new ScheduleCommand(
            m_arm.goToPosition(m_autos.closestSide().algaeArm)),
            Set.of(m_arm.mainPivotS, m_arm.elevatorS, m_arm.wristS)))
    .onTrue(m_hand.inAlgae());

    // Stow
    m_driverController.b().onTrue(m_arm.goToPosition(Arm.Positions.GROUND_ALGAE)).onTrue(m_hand.inAlgae());
    m_driverController.leftBumper().onTrue(m_arm.algaeStowWithHome());
       // m_arm.goToPosition(Arm.Positions.STOW));
    // Score coral and stow
    m_driverController.rightBumper().onTrue(
        m_hand.outCoral().withTimeout(0.5).andThen(
          new ScheduleCommand(m_arm.goToPosition(Arm.Positions.INTAKE_CORAL))
            )
    )
        ;

    // Score algae and stow if at barge position
    m_driverController.leftTrigger().onTrue(parallel(
        m_hand.outAlgae().withTimeout(0.5)).andThen(new ScheduleCommand(m_arm.goToPosition(Arm.Positions.STOW))
        .onlyIf(()->
            m_arm.getPosition().elevatorMeters()>Arm.Positions.L3.elevatorMeters())));
    // Auto align to operator selected position on reef for coral scoring
    m_driverController.rightTrigger().whileTrue(m_autos.autoScore());

    /*
     * m_driverController.leftStick().whileTrue(Commands.none());
     * m_driverController.rightStick().whileTrue(Commands.none());
     */

    // HOME arm brake
    m_driverController.leftStick().and(m_driverController.rightStick()).and(RobotModeTriggers.disabled())
        .onTrue(m_armBrakeS.home())
        .onTrue(LightStripS.top.stateC(()->TopStates.Intaked).withTimeout(0.5));

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
    if (RobotBase.isSimulation()) {
      toGoal.clear();
      toGoal.addAll(m_drivebaseS.m_repulsor.getTrajectory(
          m_drivebaseS.state().Pose.getTranslation(),
          m_drivebaseS.m_repulsor.goal().getTranslation(), 3 * 0.02));
      // This needs to be just before pdh.update() so it can't be in
      // simulationPeriodic, which is after
      // TalonFXPDHChannel.refresh();
      // TalonFXPDHChannel.currentSignalsRio.forEach((channel, signal)->{
      // PowerDistributionSim.instance.setChannelCurrent(channel,
      // signal.getValueAsDouble());}
      // );
      // TalonFXPDHChannel.currentSignalsCanivore.forEach((channel, signal)->{
      // PowerDistributionSim.instance.setChannelCurrent(channel,
      // signal.getValueAsDouble());}
      // );
    }
    // toProc.clear();
    // to_a_left.clear();
    // to_b_left.clear();
    // to_proc_stat.clear();

    // toProc.addAll(m_drivebaseS.m_repulsor.getTrajectory(m_drivebaseS.state().Pose.getTranslation(),
    // proc.getTranslation(), 3*0.02));
    // to_a_left.addAll(m_drivebaseS.m_repulsor.getTrajectory(m_drivebaseS.state().Pose.getTranslation(),
    // a_left.getTranslation(), 3*0.02));
    // to_b_left.addAll(m_drivebaseS.m_repulsor.getTrajectory(m_drivebaseS.state().Pose.getTranslation(),
    // b_left.getTranslation(), 3*0.02));
    // to_proc_stat.addAll(m_drivebaseS.m_repulsor.getTrajectory(m_drivebaseS.state().Pose.getTranslation(),
    // proc_stat.getTranslation(), 3*0.02));

    m_arm.update();
    RobotVisualizer.setArmPosition(m_arm.getPosition());
    Epilogue.talonFXLogger.refreshAll();
    // pdh.update();
    CommandScheduler.getInstance().run();
    LightStripS.periodic();
    DriverStation.getAlliance().ifPresent(alliance->{
      if (alliance == Alliance.Red) {
        LightStripS.top.requestState(TopStates.RedAlliance);
      } else {
        LightStripS.top.requestState(TopStates.BlueAlliance);
      }
    });
    if (m_autos.drivetrainSafeToAlignTrig.getAsBoolean()) {
      LightStripS.outer.requestSafeToAlign();
    }
    var loopTime = Timer.getFPGATimestamp()-lastTimestamp;
    SmartDashboard.putNumber("loopTime", loopTime);
    lastTimestamp = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("robotHeartbeat", lastRobotHeartbeat++);

  }

  private final Rotation3d coral_hand_rotation = new Rotation3d(0, Units.degreesToRadians(20), 0);

  public Pose3d getCoralPose() {
    if (m_autos.hasCoral()) {
      return new Pose3d(m_drivebaseS.getPose()).plus(new Transform3d(
          RobotVisualizer.getComponents()[3].getTranslation(),
          RobotVisualizer.getComponents()[3].getRotation())).plus(
              new Transform3d(
                  -0.04, -m_autos.getDistanceSensorOffset(), 0.18, coral_hand_rotation));
    } else {
      return Pose3d.kZero;
    }
  }

  public Pose3d getAlgaePose() {
    return new Pose3d(m_drivebaseS.getPose()).plus(new Transform3d(
        RobotVisualizer.getComponents()[3].getTranslation(),
        RobotVisualizer.getComponents()[3].getRotation())).plus(
            new Transform3d(
                0.2, 0, 0.3, Rotation3d.kZero));
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
              }).onlyWhile(DriverStation::isAutonomousEnabled)
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
    switch (m_autos.selectedReefPOI()) {
      case A:
      case B:
        LightStripS.outer.requestState(OuterStates.AB);
        break;
      case C:
      case D:
        LightStripS.outer.requestState(OuterStates.CD);
        break;
      case E:
      case F:
        LightStripS.outer.requestState(OuterStates.EF);
        break;
      case G:
      case H:
        LightStripS.outer.requestState(OuterStates.GH);
        break;
      case I:
      case J:
        LightStripS.outer.requestState(OuterStates.IJ);
        break;
      case K:
      case L:
        LightStripS.outer.requestState(OuterStates.KL);
        break;
      default:
        LightStripS.outer.requestState(OuterStates.Default);
    }
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
