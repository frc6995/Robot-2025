package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.Choreo;
import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.operator.OperatorBoard;
import frc.robot.Arm.ArmPosition;
import frc.robot.driver.CommandOperatorKeypad;
import frc.robot.subsystems.DriveBaseS;
import frc.robot.subsystems.Hand;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ChoreoVariables;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

@Logged(strategy = Strategy.OPT_IN)
public class Autos {
  private final DriveBaseS m_drivebase;
  private final Arm m_arm;
  private final Hand m_hand;
  private final OperatorBoard m_board;
  private final AutoFactory m_autoFactory;
  public final AutoChooser m_autoChooser;

  public final HashMap<String, Supplier<Command>> autos = new HashMap<>();
  @Logged
  public final CoralSensor m_coralSensor = new CoralSensor();
  public Autos(DriveBaseS drivebase, Arm arm, Hand hand, OperatorBoard board, TrajectoryLogger<SwerveSample> trajlogger) {
    m_drivebase = drivebase;
    m_arm = arm;
    m_hand = hand;
    m_board = board;
    m_autoChooser = new AutoChooser();
    m_autoFactory =
        new AutoFactory(
            () -> m_drivebase.state().Pose,
            m_drivebase::resetOdometry,
            m_drivebase::followPath,
            true,
            m_drivebase,
            m_drivebase::logTrajectory);
    addAutos();
    new Trigger(()->DriverStation.getStickButton(4, 3)).onTrue(runOnce(this::testAutos).ignoringDisable(true));
    m_autoChooser.addRoutine("splitCheeseRoutine", this::splitPathAutoRoutine);

    m_autoChooser.addCmd("HIJKL_SL3", this::HIJKL_SL3);
  }

  public void addAutos() {
    autos.put("KK_SL3", ()->this.KK_SL3().cmd());
    autos.put("JKLA_FLEX", () -> flexAuto(POI.STJ, POI.SL3, POI.J, POI.K, POI.L, POI.A));
    autos.put( "HIJKLA_FLEX", () -> flexAuto(POI.STH, POI.SL3, POI.H, POI.I, POI.J, POI.K, POI.L, POI.A));
    autos.put( "HIJKLA_FLEX", () -> flexAuto(POI.STH, POI.SR3, POI.H, POI.I, POI.J, POI.K, POI.L, POI.A));

    for (Entry<String, Supplier<Command>> entry: autos.entrySet()) {
      m_autoChooser.addCmd(entry.getKey(), entry.getValue());
    }
  }
  private Alert successfulAutoTest = new Alert("Successfully Checked Autos", AlertType.kInfo);
  public void testAutos() {
    for (Entry<String, Supplier<Command>> entry: autos.entrySet()) {
      entry.getValue().get();
    }
    successfulAutoTest.set(true);
  }


  public AutoRoutine splitPathAutoRoutine() {
    AutoRoutine routine = m_autoFactory.newRoutine("splitPathRoutine");

    AutoTrajectory start = routine.trajectory("split_path", 0);
    AutoTrajectory secondHalf = routine.trajectory("split_path", 1);

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            sequence(
                start.resetOdometry(),
                start.cmd(),
                m_drivebase.stop().withTimeout(2.54),
                secondHalf.cmd()));

    return routine;
  }

  public AutoRoutine JKL_SL3() {
    var routine = m_autoFactory.newRoutine("JKL_SL3");
    AutoTrajectory J_SL3 = routine.trajectory("SL3-J", 1);
    AutoTrajectory SL3_K = routine.trajectory("SL3-K", 0);
    AutoTrajectory K_SL3 = routine.trajectory("SL3-K", 1);
    AutoTrajectory SL3_L = routine.trajectory("SL3-L", 0);

    routine
        .active()
        .onTrue(
            sequence(J_SL3.resetOdometry(), J_SL3.cmd(), SL3_K.cmd(), K_SL3.cmd(), SL3_L.cmd()));
    return routine;
  }

  /**
   * Requires: only drivetrain Runs: drivetrain and rollers
   *
   * @param target
   * @param outtakeSeconds
   * @return
   */
  public Command alignAndDrop(
      Optional<Pose2d> target, ArmPosition position, double outtakeSeconds) {
    return target
        .map((pose) -> alignAndDrop(() -> pose, position, outtakeSeconds))
        .orElse(Commands.none());
  }

  public Command alignAndDrop(
      Supplier<Pose2d> target, ArmPosition position, double outtakeSeconds) {
    return race(
        waitUntil(
                m_drivebase.atPose(target)
                    .and(
                        () -> {
                          return m_arm.atPosition(position);
                        }))
            .andThen(outtake().withTimeout(outtakeSeconds).asProxy()), // Proxy so hand isn't directly required
        m_drivebase.pidToPoseC(target));
  }
  @Logged
  public double getDistanceSensorOffset() {
    return m_coralSensor.distanceOffset();
  }
  @Logged
  public boolean hasCoral() {
    return m_coralSensor.hasCoral();
  }
  public Supplier<Pose2d> sensorOffsetPose(Supplier<Pose2d> original) {
    // TODO reduce allocations
    // TODO angle instead of sideways? 
    return () ->
        original
            .get()
            .plus(new Transform2d(new Translation2d(0, getDistanceSensorOffset()), Rotation2d.kZero));
  }

  private POI selectedReefPOI() {
    return switch(m_board.getBranch()) {
      case 0 -> POI.A;
      case 1 -> POI.B;
      case 2 -> POI.C;
      case 3 -> POI.D;
      case 4 -> POI.E;
      case 5 -> POI.F;
      case 6 -> POI.G;
      case 7 -> POI.H;
      case 8 -> POI.I;
      case 9 -> POI.J;
      case 10 -> POI.K;
      case 11 -> POI.L;
      default -> POI.A;
    };
  }
  public Pose2d selectedReefPose() {
    return selectedReefPOI().flippedPose();
  }
  public Supplier<Pose2d> offsetSelectedReefPose = sensorOffsetPose(this::selectedReefPose);
  @Logged
  public Pose2d offsetSelectedReefPoseLog() {
    return offsetSelectedReefPose.get();
  }

  public Command alignToSelectedPose() {
    return m_drivebase.repulsorCommand(offsetSelectedReefPose);
  }

  private ArmPosition selectedBranch() {
    return switch (m_board.getLevel()) {
      case 0-> Arm.Positions.STOW;
      case 1-> Arm.Positions.L2;
      case 2-> Arm.Positions.L3;
      case 3-> Arm.Positions.L4;
      default-> Arm.Positions.STOW;
    };
  }

  public Command autoScore() {
    var target = offsetSelectedReefPose;
    return defer(
      ()->{
        return deadline(
          waitUntil(
            m_drivebase.atPose(target)
                .and(
                    () -> m_arm.atPosition(selectedBranch())))
          .andThen(outtake().withTimeout(AUTO_OUTTAKE_TIME).asProxy())
          ,
          m_drivebase.pidToPoseC(offsetSelectedReefPose).asProxy(),
          m_arm.goToPosition(selectedBranch()).asProxy()
        ).andThen(
          new ScheduleCommand(m_arm.goToPosition(Arm.Positions.STOW))
        );
      },Set.of());
    
  }

  public Command outtake() {
    return m_hand.out().alongWith(runOnce(()->m_coralSensor.setHasCoral(false)));
  }

  public Command flexAuto(POI start, POI intake, POI firstScore, POI... rest)
      throws NoSuchElementException {
    var routine = m_autoFactory.newRoutine("JKLA_SL3");
    var start_first = start.toChecked(firstScore, routine).map(this::bindL4).get();
    var start_first_final = start_first.getFinalPose().get();
    var first_intake = firstScore.toChecked(intake, routine).map(this::bindIntake).get();
    // Set up the start->first score -> intake 
    routine
        .active()
        .onTrue(
            sequence(
                start_first.resetOdometry(),
                start_first
                    .cmd(),
                    // .until(
                    //     start_first.atTranslation(
                    //         firstScore.bluePose.getTranslation(), Units.inchesToMeters(24))),
                alignAndDrop(
                        sensorOffsetPose(() -> start_first_final),
                        Arm.Positions.L4,
                        AUTO_OUTTAKE_TIME)
                    .onlyWhile(routine.active()),
                first_intake.cmd()));
    first_intake.atTime(0.1).onTrue(m_hand.in());
    var toIntake = first_intake;
    for (POI poi : rest) {
      if (RobotBase.isSimulation()) {
        toIntake.done(50).onTrue(runOnce(()->m_coralSensor.setHasCoral(true)));
      }
      var toReef = intake.toChecked(poi, routine).map(this::bindL4).get();
      var toReefFinal = toReef.getFinalPose().get();
      var nextToIntake = poi.toChecked(intake, routine).map(this::bindIntake).get();
      toIntake
          .done()
          .onTrue(
              sequence(
                  deadline(
                    waitUntil(this::hasCoral),
                    m_drivebase.pidToPoseC(toReef.getInitialPose())
                  ),
                  toReef.cmd().until(
                    toReef.atTranslation(
                        poi.bluePose.getTranslation(), Units.inchesToMeters(24))),
                  alignAndDrop(
                          sensorOffsetPose(() -> toReefFinal), Arm.Positions.L4, AUTO_OUTTAKE_TIME)
                      .onlyWhile(routine.active()),
                  nextToIntake.cmd()));
      toIntake = nextToIntake;
    }
    return routine.cmd();
  }

  public Command JKLA_SL3() {
    var routine = m_autoFactory.newRoutine("JKLA_SL3");
    var STJ_J = POI.STJ.to(POI.J, routine).map(this::bindL4).get();
    var J_SL3 = POI.J.to(POI.SL3, routine).map(this::bindIntake).get();
    var SL3_K = POI.SL3.to(POI.K, routine).map(this::bindL4).get();
    var K_SL3 = POI.K.to(POI.SL3, routine).map(this::bindIntake).get();
    var SL3_L = POI.SL3.to(POI.L, routine).map(this::bindL4).get();
    var L_SL3 = POI.L.to(POI.SL3, routine).map(this::bindIntake).get();
    var SL3_A = POI.SL3.to(POI.A, routine).map(this::bindL4).get();
    routine
        .active()
        .onTrue(
            sequence(
                STJ_J.resetOdometry(),
                STJ_J.cmd(),
                alignAndDrop(STJ_J.getFinalPose(), Arm.Positions.L4, AUTO_OUTTAKE_TIME),
                J_SL3.cmd(),
                SL3_K.cmd(),
                alignAndDrop(SL3_K.getFinalPose(), Arm.Positions.L4, AUTO_OUTTAKE_TIME),
                K_SL3.cmd(),
                SL3_L.cmd(),
                alignAndDrop(SL3_L.getFinalPose(), Arm.Positions.L4, AUTO_OUTTAKE_TIME),
                L_SL3.cmd(),
                SL3_A.cmd(),
                alignAndDrop(SL3_A.getFinalPose(), Arm.Positions.L4, AUTO_OUTTAKE_TIME),
                new ScheduleCommand(m_arm.goToPosition(Arm.Positions.INTAKE))));
    return routine.cmd();
  }

  private final double TIME_INTAKE_TO_L4 = 1.4;
  private final double AUTO_OUTTAKE_TIME = 0.25;

  private AutoTrajectory bindL4(AutoTrajectory trajectory) {
    trajectory
        .atTime(Math.max(0, trajectory.getRawTrajectory().getTotalTime() - TIME_INTAKE_TO_L4))
        .onTrue(m_arm.goToPosition(Arm.Positions.L4));
    return trajectory;
  }

  private AutoTrajectory bindIntake(AutoTrajectory trajectory) {
    trajectory.atTime(0).onTrue(m_arm.goToPosition(Arm.Positions.INTAKE));
    return trajectory;
  }

  public AutoRoutine KK_SL3() {
    var routine = m_autoFactory.newRoutine("KK_SL3");
    AutoTrajectory K_SL3 = bindIntake(routine.trajectory("SL3-K", 1));
    AutoTrajectory SL3_K = bindL4(routine.trajectory("SL3-K", 0));

    routine.active().onTrue(sequence(K_SL3.resetOdometry(), K_SL3.cmd(), SL3_K.cmd()));
    return routine;
  }

  public Command HIJKL_SL3() {
    var routine = m_autoFactory.newRoutine("HIJKL_SL3");
    var STH_H = bindL4(routine.trajectory("STH-H", 0));
    var H_SL3 = bindIntake(routine.trajectory("SL3-H", 1));
    var SL3_I = bindL4(routine.trajectory("SL3-I", 0));
    var I_SL3 = bindIntake(routine.trajectory("SL3-I", 1));
    AutoTrajectory SL3_J = bindL4(routine.trajectory("SL3-J", 0));
    AutoTrajectory J_SL3 = bindIntake(routine.trajectory("SL3-J", 1));
    AutoTrajectory SL3_K = bindL4(routine.trajectory("SL3-K", 0));
    AutoTrajectory K_SL3 = bindIntake(routine.trajectory("SL3-K", 1));
    AutoTrajectory SL3_L = bindL4(routine.trajectory("SL3-L", 0));
    AutoTrajectory L_SL3 = bindIntake(routine.trajectory("SL3-L", 1));
    AutoTrajectory SL3_A = bindL4(routine.trajectory("SL3-A", 0));
    routine
        .active()
        .onTrue(
            sequence(
                STH_H.resetOdometry(),
                STH_H.cmd(),
                alignAndDrop(STH_H.getFinalPose(), Arm.Positions.L4, AUTO_OUTTAKE_TIME),
                H_SL3.cmd(),
                SL3_I.cmd(),
                alignAndDrop(SL3_I.getFinalPose(), Arm.Positions.L4, AUTO_OUTTAKE_TIME),
                I_SL3.cmd(),
                SL3_J.cmd(),
                alignAndDrop(SL3_J.getFinalPose(), Arm.Positions.L4, AUTO_OUTTAKE_TIME),
                J_SL3.cmd(),
                SL3_K.cmd(),
                alignAndDrop(SL3_K.getFinalPose(), Arm.Positions.L4, AUTO_OUTTAKE_TIME),
                K_SL3.cmd(),
                SL3_L.cmd(),
                alignAndDrop(SL3_L.getFinalPose(), Arm.Positions.L4, AUTO_OUTTAKE_TIME),
                L_SL3.cmd(),
                SL3_A.cmd(),
                alignAndDrop(SL3_A.getFinalPose(), Arm.Positions.L4, AUTO_OUTTAKE_TIME),
                new ScheduleCommand(m_arm.goToPosition(Arm.Positions.INTAKE))));
    return routine.cmd();
  }
}
