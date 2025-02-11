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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.operator.OperatorBoard;
import frc.robot.Arm.ArmPosition;
import frc.robot.driver.CommandOperatorKeypad;
import frc.robot.subsystems.ClimbHookS;
import frc.robot.subsystems.DriveBaseS;
import frc.robot.subsystems.Hand;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ChoreoVariables;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Logged(strategy = Strategy.OPT_IN)
public class Autos {
  private final DriveBaseS m_drivebase;
  private final Arm m_arm;
  private final Hand m_hand;
  private final OperatorBoard m_board;
  private final AutoFactory m_autoFactory;
  public final AutoChooser m_autoChooser;
  public final ClimbHookS m_ClimbHookS;

  public Autos(DriveBaseS drivebase, Arm arm, Hand hand, OperatorBoard board, ClimbHookS ClimbHookS, TrajectoryLogger<SwerveSample> trajlogger) {
    m_drivebase = drivebase;
    m_arm = arm;
    m_hand = hand;
    m_board = board;
    m_ClimbHookS  = ClimbHookS;
    m_autoChooser = new AutoChooser();
    m_autoFactory =
        new AutoFactory(
            () -> m_drivebase.state().Pose,
            m_drivebase::resetOdometry,
            m_drivebase::followPath,
            true,
            m_drivebase,
            m_drivebase::logTrajectory);

    // add autos to the chooser
    // m_autoChooser.addCmd("testPath", this::testPath);
    // m_autoChooser.addRoutine("testPathRoutine", this::testPathRoutine);
    m_autoChooser.addRoutine("splitCheeseRoutine", this::splitPathAutoRoutine);
    m_autoChooser.addRoutine("KK_SL3", this::KK_SL3);
    // m_autoChooser.addRoutine("JKL_SL3", this::JKL_SL3);
    m_autoChooser.addCmd("JKLA_SL3", this::JKLA_SL3);
    m_autoChooser.addCmd("JKLA_FLEX", () -> flexAuto(POI.STJ, POI.SL3, POI.J, POI.K, POI.L, POI.A));
    m_autoChooser.addCmd(
        "HIJKLA_FLEX", () -> flexAuto(POI.STH, POI.SL3, POI.H, POI.I, POI.J, POI.K, POI.L, POI.A));
    m_autoChooser.addCmd("HIJKL_SL3", this::HIJKL_SL3);
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

  public double toleranceMeters = Units.inchesToMeters(1);
  public double toleranceRadians = Units.degreesToRadians(2);

  private boolean withinTolerance(Rotation2d lhs, Rotation2d rhs, double toleranceRadians) {
    if (Math.abs(toleranceRadians) > Math.PI) {
      return true;
    }
    double dot = lhs.getCos() * rhs.getCos() + lhs.getSin() * rhs.getSin();
    // cos(θ) >= cos(tolerance) means |θ| <= tolerance, for tolerance in [-pi, pi],
    // as pre-checked
    // above.
    return dot > Math.cos(toleranceRadians);
  }

  public Trigger atPose(Supplier<Pose2d> poseSup) {
    return new Trigger(
        () -> {
          Pose2d pose = poseSup.get();
          Pose2d currentPose = m_drivebase.getPose();
          boolean transValid =
              currentPose.getTranslation().getDistance(pose.getTranslation()) < toleranceMeters;
          boolean rotValid =
              withinTolerance(currentPose.getRotation(), pose.getRotation(), toleranceRadians);
          return transValid && rotValid;
        });
  }

  public Trigger atPose(Optional<Pose2d> poseOpt) {
    return poseOpt.map(this::atPose).orElse(new Trigger(() -> false));
  }

  public Trigger atPose(Pose2d pose) {
    return atPose(() -> pose);
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
                atPose(target)
                    .and(
                        () -> {
                          return m_arm.atPosition(position);
                        }))
            .andThen(m_hand.out().withTimeout(outtakeSeconds).asProxy()), // Proxy so hand isn't directly required
        m_drivebase.pidToPoseC(target));
  }
  @Logged
  public double getDistanceSensorOffset() {
    return -0.1;
  }
  @Logged
  public boolean hasCoral() {
    return getDistanceSensorOffset() > -0.2;
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
            atPose(target)
                .and(
                    () -> m_arm.atPosition(selectedBranch())))
          .andThen(m_hand.out().withTimeout(AUTO_OUTTAKE_TIME).asProxy())
          ,
          m_drivebase.pidToPoseC(offsetSelectedReefPose).asProxy(),
          m_arm.goToPosition(selectedBranch()).asProxy()
        ).andThen(
          m_arm.goToPosition(Arm.Positions.STOW).asProxy()
        );
      },Set.of());
    
  }

  private static List<String> TRAJECTORIES = List.of(Choreo.availableTrajectories());

  public enum POI {
    A(true),
    B(true),
    C(true),
    D(true),
    E(true),
    F(true),
    G(true),
    H(true),
    I(true),
    J(true),
    K(true),
    L(true),
    SL3(false),
    STJ(false),
    STI(false),
    STH(false);
    public final Pose2d bluePose;
    public final Pose2d redPose;
    public final boolean isReef;

    private POI(boolean isReef) {
      this.isReef = isReef;
      bluePose = ChoreoVariables.getPose(this.name());
      redPose = AllianceFlipUtil.flip(bluePose);
    }
    private POI(boolean isReef, Pose2d bluePose) {
      this.isReef = isReef;
      this.bluePose = bluePose;
      redPose = AllianceFlipUtil.flip(bluePose);
    }
    public Pose2d flippedPose() {
      return AllianceFlipUtil.shouldFlip() ? redPose : bluePose;
    }
    public Optional<AutoTrajectory> to(POI next, AutoRoutine routine) {
      String name;
      boolean reverse;
      if (this.isReef && !next.isReef) {
        name = next.name() + "-" + this.name();
        if (!TRAJECTORIES.contains(name)) {
          return Optional.empty();
        }
        reverse = true;
      } else if (!this.isReef && next.isReef) {
        name = this.name() + "-" + next.name();
        reverse = false;
      } else {
        return Optional.empty();
      }
      return Optional.of(routine.trajectory(name, reverse ? 1 : 0));
    }
  }

  public Command flexAuto(POI start, POI intake, POI firstScore, POI... rest)
      throws NoSuchElementException {
    var routine = m_autoFactory.newRoutine("JKLA_SL3");
    var start_first = start.to(firstScore, routine).map(this::bindL4).get();
    var start_first_final = start_first.getFinalPose().get();
    var first_intake = firstScore.to(intake, routine).map(this::bindIntake).get();
    // Set up the start->first score -> intake 
    routine
        .active()
        .onTrue(
            sequence(
                start_first.resetOdometry(),
                start_first
                    .cmd()
                    .until(
                        start_first.atTranslation(
                            firstScore.bluePose.getTranslation(), Units.inchesToMeters(24))),
                alignAndDrop(
                        sensorOffsetPose(() -> start_first_final),
                        Arm.Positions.L4,
                        AUTO_OUTTAKE_TIME)
                    .onlyWhile(routine.active()),
                first_intake.cmd()));
    var toIntake = first_intake;
    for (POI poi : rest) {
      var toReef = intake.to(poi, routine).map(this::bindL4).get();
      var toReefFinal = toReef.getFinalPose().get();
      var nextToIntake = poi.to(intake, routine).map(this::bindIntake).get();
      toIntake
          .done()
          .onTrue(
              sequence(
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
