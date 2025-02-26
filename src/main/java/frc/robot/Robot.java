// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.operator.OperatorBoard;
import frc.operator.RealOperatorBoard;
import frc.operator.SimOperatorBoard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.driver.CommandOperatorKeypad;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmBrakeS;
import frc.robot.subsystems.ClimbHookS;
// import frc.robot.logging.TalonFXLogger;
import frc.robot.subsystems.DriveBaseS;
import frc.robot.subsystems.RealHandS;
import frc.robot.subsystems.NoneHandS;
import frc.robot.subsystems.Hand;
import frc.robot.NoneArm;
import frc.robot.util.AlertsUtil;
import java.util.ArrayList;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends TimedRobot {
  // public PDData pdh = PDData.create(1, ModuleType.kRev);
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final OperatorBoard m_operatorBoard = Robot.isReal() ? new RealOperatorBoard(1) : new SimOperatorBoard(1);
  private final DriveBaseS m_drivebaseS = TunerConstants.createDrivetrain();
  private final RealArm m_arm = new RealArm();
  private final Hand m_hand = RobotBase.isReal() ?  new NoneHandS() : new RealHandS();
  private final ClimbHookS m_climbHookS = new ClimbHookS();
  private final ArmBrakeS m_armBrakeS = new ArmBrakeS();
  private final Autos m_autos = new Autos(m_drivebaseS, m_arm, m_hand, m_operatorBoard, m_climbHookS, m_armBrakeS, (traj, isStarting) -> {});
  private final SwerveRequest.FieldCentric m_driveRequest = new FieldCentric();

  // private final CommandOperatorKeypad m_keypad = new CommandOperatorKeypad(5);
  // private final DrivetrainSysId m_driveId = new DrivetrainSysId(m_drivebaseS);

  private Mechanism2d VISUALIZER;
  public Pose3d[] components() {return RobotVisualizer.getComponents();}

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    VISUALIZER = RobotVisualizer.MECH_VISUALIZER;
    Epilogue.bind(this);
    AlertsUtil.bind(
        new Alert("Driver Xbox Disconnect", AlertType.kError),
        () -> !m_driverController.isConnected());
    m_drivebaseS.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivebaseS.applyRequest(
            () ->
                DriverStation.isAutonomous()
                    ? m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
                    : m_driveRequest
                        .withVelocityX(
                            -m_driverController.getLeftY()
                                * 4) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            -m_driverController.getLeftX() * 4) // Drive left with negative X (left)
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
    
    boolean doingSysId = false;
    
    RobotModeTriggers.autonomous().whileTrue(m_autos.m_autoChooser.selectedCommandScheduler());
  }

  public void configureDriverController() {
    //TODO: assign buttons to functions specified in comments

    // intake algae from ground (move arm to ground position, intake and stow)
    m_driverController.a().whileTrue(m_autos.autoScore());
    // Drive and autoalign to processor
    m_driverController.b().whileTrue(m_arm.goToPosition(Arm.Positions.INTAKE));
    // Drive and autoalign to barge
    m_driverController.y().whileTrue(m_arm.goToPosition(Arm.Positions.STOW));
    // Intake algae from reef (autoalign, move arm to possition, intake and stow)
    m_driverController.x().whileTrue(m_autos.alignToSelectedPose());

    // Drive and autoalign to edge of coral station L (after this, use d-pad to drive relative)
    m_driverController.leftBumper().whileTrue(Commands.none());
    // Drive and autoalign to edge of coral station L (after this, use d-pad to drive relative)
    m_driverController.rightBumper().whileTrue(Commands.none());

    // score algae
    m_driverController.leftTrigger().whileTrue(Commands.none());
    // Auto align to operator selected position on reef for coral scoring
    m_driverController.rightTrigger().whileTrue(Commands.none());

    /*
    m_driverController.leftStick().whileTrue(Commands.none());
    m_driverController.rightStick().whileTrue(Commands.none());
    */

    // autoalign to deep cage
    m_driverController.back().whileTrue(Commands.none());
    // execute cage climb
    m_driverController.start().whileTrue(Commands.none());

    // TODO: dpad robot relative driving
  }



  ArrayList<Translation2d> toGoal = new ArrayList<>();
  Pose3d emptyPose = Pose3d.kZero;
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_operatorBoard.poll();
    if (RobotBase.isSimulation()) {
      toGoal.clear();
      toGoal.addAll(m_drivebaseS.m_repulsor.getTrajectory(
        m_drivebaseS.state().Pose.getTranslation(),
        m_drivebaseS.m_repulsor.goal().getTranslation(), 3*0.02));
      // This needs to be just before pdh.update() so it can't be in simulationPeriodic, which is after 
      // TalonFXPDHChannel.refresh();
      // TalonFXPDHChannel.currentSignalsRio.forEach((channel, signal)->{
      //   PowerDistributionSim.instance.setChannelCurrent(channel, signal.getValueAsDouble());}
      //   );
      // TalonFXPDHChannel.currentSignalsCanivore.forEach((channel, signal)->{
      //   PowerDistributionSim.instance.setChannelCurrent(channel, signal.getValueAsDouble());}
      //   );
    }
    // toProc.clear();
    // to_a_left.clear();
    // to_b_left.clear();
    // to_proc_stat.clear();

    // toProc.addAll(m_drivebaseS.m_repulsor.getTrajectory(m_drivebaseS.state().Pose.getTranslation(), proc.getTranslation(), 3*0.02));
    // to_a_left.addAll(m_drivebaseS.m_repulsor.getTrajectory(m_drivebaseS.state().Pose.getTranslation(), a_left.getTranslation(), 3*0.02));
    // to_b_left.addAll(m_drivebaseS.m_repulsor.getTrajectory(m_drivebaseS.state().Pose.getTranslation(), b_left.getTranslation(), 3*0.02));
    // to_proc_stat.addAll(m_drivebaseS.m_repulsor.getTrajectory(m_drivebaseS.state().Pose.getTranslation(), proc_stat.getTranslation(), 3*0.02));

    m_arm.update();
    RobotVisualizer.setArmPosition(m_arm.position);
    Epilogue.talonFXLogger.refreshAll();
    // pdh.update();
    CommandScheduler.getInstance().run();
  }

  private final Rotation3d coral_hand_rotation = new Rotation3d(0, Units.degreesToRadians(105), 0);
  public Pose3d getCoralPose() {
    if (m_autos.hasCoral()) {
    return new Pose3d(m_drivebaseS.getPose()).plus(new Transform3d(
      RobotVisualizer.getComponents()[3].getTranslation(),
      RobotVisualizer.getComponents()[3].getRotation()
    )).plus(
      new Transform3d(
        -0.12, -m_autos.getDistanceSensorOffset(), 0.1, coral_hand_rotation
      )
    );
    }
    else {
      return Pose3d.kZero;
    }
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
      Commands.waitSeconds(15.3)
          .andThen(
              () -> {
                DriverStationSim.setEnabled(false);
                DriverStationSim.notifyNewData();
              })
          .schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
