package frc.robot;

import choreo.Choreo;
import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveBaseS;
import frc.robot.subsystems.DriveBaseS;

public class Autos {
    private DriveBaseS m_drivebase;

    private AutoFactory m_autoFactory;
    public AutoChooser m_autoChooser;

    public Autos(DriveBaseS drivebase, TrajectoryLogger<SwerveSample> trajlogger) {
        m_drivebase = drivebase;
        m_autoChooser = new AutoChooser();
        m_autoFactory = new AutoFactory(
            ()->m_drivebase.state().Pose, 
            m_drivebase::resetPose, 
            m_drivebase::followPath, 
            true, 
            m_drivebase, 
            trajlogger);

        // add autos to the chooser
        m_autoChooser.addCmd("testPath", this::testPath);
        m_autoChooser.addRoutine("testPathRoutine", this::testPathRoutine);
    }

    public Command testPath(){ return m_autoFactory.trajectoryCmd("test_path");}

    public AutoRoutine testPathRoutine() {
        AutoRoutine routine = m_autoFactory.newRoutine("testPathRoutine");

        AutoTrajectory testTrajectory = routine.trajectory("test_path");

        // When the routine begins, reset odometry and start the first trajectory 
        routine.active().onTrue(
            Commands.sequence(
                testTrajectory.resetOdometry(),
                testTrajectory.cmd()
            )
        );

        return routine;
    }
}
