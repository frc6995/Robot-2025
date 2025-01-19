// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.driver.CommandOperatorKeypad;
import frc.robot.generated.TunerConstants;
import frc.robot.logging.PDData;
import frc.robot.logging.PowerDistributionSim;
import frc.robot.logging.PowerDistributionSim.Channel;
// import frc.robot.logging.TalonFXLogger;
import frc.robot.logging.TalonFXPDHChannel;
import frc.robot.subsystems.DriveBaseS;
import frc.robot.subsystems.DrivetrainSysId;
import frc.robot.subsystems.AlgaePivotS;
import frc.robot.util.AlertsUtil;
import frc.robot.util.AllianceFlipUtil;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends TimedRobot {
  // public PDData pdh = PDData.create(1, ModuleType.kRev);
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final DriveBaseS m_drivebaseS = TunerConstants.createDrivetrain();
  //private final AlgaePivotS m_algaePivotS = new AlgaePivotS();

  private final Autos m_autos = new Autos(m_drivebaseS, (traj, isStarting)->{});
  private final SwerveRequest.FieldCentric m_driveRequest = new FieldCentric();
  private final CommandOperatorKeypad m_keypad = new CommandOperatorKeypad(5);
  // private final DrivetrainSysId m_driveId = new DrivetrainSysId(m_drivebaseS);

  private Mechanism2d VISUALIZER;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    VISUALIZER = RobotVisualizer.MECH_VISUALIZER;
    Epilogue.bind(this);
    AlertsUtil.bind(new Alert("Driver Xbox Disconnect", AlertType.kError), ()->!m_driverController.isConnected());
    m_drivebaseS.setDefaultCommand(
      // Drivetrain will execute this command periodically
      m_drivebaseS.applyRequest(() -> 
          DriverStation.isAutonomous() ? 
          m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0) :
          m_driveRequest.withVelocityX(-m_driverController.getLeftY() * 4) // Drive forward with negative Y (forward)
              .withVelocityY(-m_driverController.getLeftX() * 4) // Drive left with negative X (left)
              .withRotationalRate(-m_driverController.getRightX() * 2 * Math.PI) // Drive counterclockwise with negative X (left)
      )
    );
    RobotVisualizer.setupVisualizer();
    //RobotVisualizer.addAlgaeIntake(m_algaePivotS.ALGAE_PIVOT);
    SmartDashboard.putData("visualizer", VISUALIZER);

    SmartDashboard.putData("autoChooser", m_autos.m_autoChooser);
    // m_driverController.a().onTrue(m_algaePivotS.deploy());
    // m_driverController.b().onTrue(m_algaePivotS.retract());
    // m_driverController.a().whileTrue(m_drivebaseS.repulsorCommand(()->proc));
    Pose2d k = new Pose2d(3.98, 5.23, new Rotation2d(2.09));
    Pose2d i = new Pose2d(5.28, 5.07, new Rotation2d(1.05));
    m_driverController.b().whileTrue(m_drivebaseS.driveToPoseC(AllianceFlipUtil.getFlipped(k)));
    m_driverController.x().whileTrue(m_drivebaseS.driveToPoseC(AllianceFlipUtil.getFlipped(i)));
    m_driverController.y().whileTrue(m_drivebaseS.repulsorCommand(AllianceFlipUtil.getFlipped(i)));
    m_driverController.a().whileTrue(m_drivebaseS.repulsorCommand(AllianceFlipUtil.getFlipped(k)));
    boolean doingSysId = false;
    // if (doingSysId) {
    // SignalLogger.start();
    // m_keypad.key(CommandOperatorKeypad.Button.kLowLeft).whileTrue(m_driveId.sysIdTranslationDynamic(Direction.kForward));
    // m_keypad.key( CommandOperatorKeypad.Button.kMidLeft).whileTrue(m_driveId.sysIdTranslationDynamic(Direction.kReverse));
    // m_keypad.key( CommandOperatorKeypad.Button.kHighLeft).whileTrue(m_driveId.sysIdTranslationQuasistatic(Direction.kForward));
    // m_keypad.key(CommandOperatorKeypad.Button.kLeftGrid).whileTrue(m_driveId.sysIdTranslationQuasistatic(Direction.kReverse));
    // // Rotation
    // m_keypad.key(CommandOperatorKeypad.Button.kLowCenter).whileTrue(m_driveId.sysIdRotationDynamic(Direction.kForward));
    // m_keypad.key( CommandOperatorKeypad.Button.kMidCenter).whileTrue(m_driveId.sysIdRotationDynamic(Direction.kReverse));
    // m_keypad.key( CommandOperatorKeypad.Button.kHighCenter).whileTrue(m_driveId.sysIdRotationQuasistatic(Direction.kForward));
    // m_keypad.key(CommandOperatorKeypad.Button.kCenterGrid).whileTrue(m_driveId.sysIdRotationQuasistatic(Direction.kReverse));

    // m_keypad.key(CommandOperatorKeypad.Button.kLowRight).whileTrue(m_driveId.sysIdSteerDynamic(Direction.kForward));
    // m_keypad.key( CommandOperatorKeypad.Button.kMidRight).whileTrue(m_driveId.sysIdSteerDynamic(Direction.kReverse));
    // m_keypad.key( CommandOperatorKeypad.Button.kHighRight).whileTrue(m_driveId.sysIdSteerQuasistatic(Direction.kForward));
    // m_keypad.key(CommandOperatorKeypad.Button.kRightGrid).whileTrue(m_driveId.sysIdSteerQuasistatic(Direction.kReverse));
    // }
    RobotModeTriggers.autonomous().whileTrue(m_autos.m_autoChooser.selectedCommandScheduler());
  }

  ArrayList<Translation2d> toGoal = new ArrayList<>();
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
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

    for (Integer i : Epilogue.talonFXLogger.talons.keySet()){
      var object = Epilogue.talonFXLogger.talons.get(i);
      BaseStatusSignal.refreshAll(object.statorCurrent(), object.torqueCurrent(), object.position());
      //PowerDistributionSim.instance.setChannelCurrent(TalonFXPDHChannel.channels.getOrDefault(i, Channel.c00), object.supplyCurrent().getValueAsDouble());
    }
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
  public void autonomousInit() {
    
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
  public void simulationPeriodic() {

  }
}
