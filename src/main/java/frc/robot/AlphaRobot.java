// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.operator.OperatorBoard;
import frc.operator.RealOperatorBoard;
import frc.operator.SimOperatorBoard;
import frc.robot.driver.CommandOperatorKeypad;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmBrakeS;
import frc.robot.subsystems.ClimbHookS;
// import frc.robot.logging.TalonFXLogger;
import frc.robot.subsystems.DriveBaseS;
import frc.robot.subsystems.NoneHandS;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.NoneArm;
import frc.robot.util.AlertsUtil;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class AlphaRobot extends TimedRobot {
  // public PDData pdh = PDData.create(1, ModuleType.kRev);
  private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final OperatorBoard m_operatorBoard = Robot.isReal() ? new RealOperatorBoard(1) : new SimOperatorBoard(1);
  private final DriveBaseS m_drivebaseS = TunerConstants.createAlphaDrivetrain();
  private final NoneArm m_arm = new NoneArm();
  private final NoneHandS m_hand = new NoneHandS();
  private final ClimbHookS m_climbHookS = new ClimbHookS();
  private final ArmBrakeS m_armBrakeS = new ArmBrakeS();
  private final Autos m_autos = new Autos(m_drivebaseS, m_arm, m_hand, m_operatorBoard, m_climbHookS, m_armBrakeS, (traj, isStarting) -> {});
  private final SwerveRequest.FieldCentric m_driveRequest = new FieldCentric();

  private final CommandOperatorKeypad m_keypad = new CommandOperatorKeypad(5);
  // private final DrivetrainSysId m_driveId = new DrivetrainSysId(m_drivebaseS);

  private Mechanism2d VISUALIZER;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public AlphaRobot() {
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

    RobotVisualizer.addArmPivot(m_arm.getMechanism());
    // RobotVisualizer.addAlgaeIntake(m_algaePivotS.ALGAE_PIVOT);
    SmartDashboard.putData("visualizer", VISUALIZER);

    SmartDashboard.putData("autoChooser", m_autos.m_autoChooser);

    m_driverController.a().whileTrue(m_arm.goToPosition(Arm.Positions.L4));
    m_driverController.b().whileTrue(m_arm.goToPosition(Arm.Positions.STOW));
    RobotModeTriggers.autonomous().whileTrue(m_autos.m_autoChooser.selectedCommandScheduler());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Epilogue.talonFXLogger.refreshAll();
    // pdh.update();
    CommandScheduler.getInstance().run();
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
  public void autonomousInit() {}

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
