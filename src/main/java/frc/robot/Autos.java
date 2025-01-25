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
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.DriveBaseS;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Autos {
    private DriveBaseS m_drivebase;
    private Arm m_arm;

    private AutoFactory m_autoFactory;
    public AutoChooser m_autoChooser;

    public Autos(DriveBaseS drivebase, Arm arm, TrajectoryLogger<SwerveSample> trajlogger) {
        m_drivebase = drivebase;
        m_arm = arm;
        m_autoChooser = new AutoChooser();
        m_autoFactory = new AutoFactory(
            ()->m_drivebase.state().Pose, 
            m_drivebase::resetOdometry, 
            m_drivebase::followPath, 
            true, 
            m_drivebase, 
            m_drivebase::logTrajectory);

        // add autos to the chooser
        // m_autoChooser.addCmd("testPath", this::testPath);
        // m_autoChooser.addRoutine("testPathRoutine", this::testPathRoutine);
        // m_autoChooser.addRoutine("splitCheeseRoutine", this::splitPathAutoRoutine);
        m_autoChooser.addRoutine("KK_SL3", this::KK_SL3);
        // m_autoChooser.addRoutine("JKL_SL3", this::JKL_SL3);
        // m_autoChooser.addRoutine("JKLA_SL3", this::JKLA_SL3);
        
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

    public AutoRoutine splitPathAutoRoutine() {
        AutoRoutine routine = m_autoFactory.newRoutine("splitPathRoutine");

        AutoTrajectory start = routine.trajectory("split_path", 0);
        AutoTrajectory secondHalf = routine.trajectory("split_path", 1);

        // When the routine begins, reset odometry and start the first trajectory 
        routine.active().onTrue(
            sequence(
                start.resetOdometry(),
                start.cmd(),
                m_drivebase.stop().withTimeout(2.54),
                secondHalf.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine JKL_SL3() {
        var routine = m_autoFactory.newRoutine("JKL_SL3");
        AutoTrajectory J_SL3 = routine.trajectory("SL3-J", 1);
        AutoTrajectory SL3_K = routine.trajectory("SL3-K", 0);
        AutoTrajectory K_SL3 = routine.trajectory("SL3-K", 1);
        AutoTrajectory SL3_L = routine.trajectory("SL3-L", 0);

        routine.active().onTrue(
            sequence(
                J_SL3.resetOdometry(),
                J_SL3.cmd(),
                SL3_K.cmd(),
                K_SL3.cmd(),
                SL3_L.cmd()
            )
        );
        return routine;
    }

    public AutoRoutine JKLA_SL3() {
        var routine = m_autoFactory.newRoutine("JKLA_SL3");
        AutoTrajectory J_SL3 = routine.trajectory("SL3-J", 1);
        AutoTrajectory SL3_K = routine.trajectory("SL3-K", 0);
        AutoTrajectory K_SL3 = routine.trajectory("SL3-K", 1);
        AutoTrajectory SL3_L = routine.trajectory("SL3-L", 0);
        AutoTrajectory L_SL3 = routine.trajectory("SL3-L", 1);
        AutoTrajectory A_SL3 = routine.trajectory("SL3-A", 0);
        routine.active().onTrue(
            sequence(
                J_SL3.resetOdometry(),
                J_SL3.cmd(),
                SL3_K.cmd(),
                K_SL3.cmd(),
                SL3_L.cmd(),
                L_SL3.cmd(),
                A_SL3.cmd()
            )
        );
        return routine;
    }

    public AutoRoutine KK_SL3() {
        var routine = m_autoFactory.newRoutine("KK_SL3");
        AutoTrajectory K_SL3 = routine.trajectory("SL3-K", 1);
        AutoTrajectory SL3_K = routine.trajectory("SL3-K", 0);
        routine.active().onTrue(
            sequence(
                K_SL3.resetOdometry(),
                K_SL3.cmd(),
                SL3_K.cmd()
            )
        );
        return routine;
    }

}
