package frc.robot;

import choreo.Choreo;
import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveBaseS;
import frc.robot.subsystems.DriveBaseS;

public class Autos {
    private DriveBaseS m_drivebase;

    private AutoFactory m_autoFactory;

    public Autos(DriveBaseS drivebase, TrajectoryLogger<SwerveSample> trajlogger) {
        m_drivebase = drivebase;

        m_autoFactory = new AutoFactory(()->m_drivebase.state().Pose, 
        m_drivebase::resetPose, 
        m_drivebase::followPath, 
        true, 
        m_drivebase, 
        trajlogger);
    }

    public Command testPath(){ return m_autoFactory.trajectoryCmd("test_path");}
}
