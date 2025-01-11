// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.BaseStatusSignal;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.logging.PDData;
import frc.robot.logging.PowerDistributionSim;
import frc.robot.logging.PowerDistributionSim.Channel;
// import frc.robot.logging.TalonFXLogger;
import frc.robot.logging.TalonFXPDHChannel;
import frc.robot.subsystems.DriveBaseS;
import frc.robot.util.AlertsUtil;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends TimedRobot {
  public PDData pdh = PDData.create(1, ModuleType.kRev);
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final DriveBaseS m_drivebaseS = TunerConstants.createDrivetrain();
  private final Autos m_Autos = new Autos(m_drivebaseS, (traj, isStarting)->{});
  private final SwerveRequest.FieldCentric m_driveRequest = new FieldCentric();
  
  final Pose2d proc = new Pose2d(6.28, 0.48, Rotation2d.kCW_90deg);
  public ArrayList<Translation2d> toProc = new ArrayList<>();
  final Pose2d a_left = new Pose2d(5, 5.24, new Rotation2d(4*Math.PI/3));
  public ArrayList<Translation2d> to_a_left = new ArrayList<>();
  final Pose2d b_left = new Pose2d(5.86, 3.86, Rotation2d.kPi);
  public ArrayList<Translation2d> to_b_left = new ArrayList<>();
  final Pose2d proc_stat = new Pose2d(1.15, 7.13, new Rotation2d(2.23));
  public ArrayList<Translation2d> to_proc_stat = new ArrayList<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    Epilogue.bind(this);
    AlertsUtil.bind(new Alert("Driver Xbox Disconnect", AlertType.kError), ()->!m_driverController.isConnected());
    m_drivebaseS.setDefaultCommand(
      // Drivetrain will execute this command periodically
      m_drivebaseS.applyRequest(() -> 
          m_driveRequest.withVelocityX(-m_driverController.getLeftY() * 4) // Drive forward with negative Y (forward)
              .withVelocityY(-m_driverController.getLeftX() * 4) // Drive left with negative X (left)
              .withRotationalRate(-m_driverController.getRightX() * 2 * Math.PI) // Drive counterclockwise with negative X (left)
      )
    );
    m_driverController.a().whileTrue(m_drivebaseS.repulsorCommand(()->proc));
    m_driverController.b().whileTrue(m_drivebaseS.repulsorCommand(()->a_left));
    m_driverController.x().whileTrue(m_drivebaseS.repulsorCommand(()->b_left));
    m_driverController.y().whileTrue(m_drivebaseS.repulsorCommand(()->proc_stat));

    RobotModeTriggers.autonomous().whileTrue(m_Autos.testPath());
  }

  private static Translation2d amp = new Translation2d(2, 8);

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

    // for (Integer i : Epilogue.talonFXLogger.talons.keySet()){
    //   var object = Epilogue.talonFXLogger.talons.get(i);
    //   BaseStatusSignal.refreshAll(object.supplyCurrent(), object.position());
    //   PowerDistributionSim.instance.setChannelCurrent(TalonFXPDHChannel.channels.getOrDefault(i, Channel.c00), object.supplyCurrent().getValueAsDouble());
    // }
    pdh.update();
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
