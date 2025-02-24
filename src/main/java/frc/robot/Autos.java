package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.Choreo;
import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
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
import frc.robot.driver.CommandOperatorKeypad;
import frc.robot.subsystems.ArmBrakeS;
import frc.robot.subsystems.ClimbHookS;
import frc.robot.subsystems.DriveBaseS;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.pivot.MainPivotS.MainPivotConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ChoreoVariables;

import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
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
  public final ClimbHookS m_ClimbHookS;
  public final ArmBrakeS m_ArmBrakeS;
  @Logged
  public final CoralSensor m_coralSensor = new CoralSensor();

  public Autos(DriveBaseS drivebase, Arm arm, Hand hand, OperatorBoard board, ClimbHookS climbHookS,
      ArmBrakeS armBrakeS, TrajectoryLogger<SwerveSample> trajlogger) {
    m_drivebase = drivebase;
    m_arm = arm;
    m_hand = hand;
    m_board = board;
    m_ClimbHookS = climbHookS;
    m_ArmBrakeS = armBrakeS;
    m_autoChooser = new AutoChooser();
    m_autoFactory = new AutoFactory(
        () -> m_drivebase.state().Pose,
        m_drivebase::resetOdometry,
        m_drivebase::followPath,
        true,
        m_drivebase,
        m_drivebase::logTrajectory);
    addAutos();
    new Trigger(() -> DriverStation.getStickButton(4, 1))
        .onTrue(runOnce(() -> m_coralSensor.setHasCoral(true)).ignoringDisable(true));
    new Trigger(() -> DriverStation.getStickButton(4, 2))
        .onTrue(runOnce(() -> m_coralSensor.setHasCoral(false)).ignoringDisable(true));
    new Trigger(() -> DriverStation.getStickButton(4, 3)).onTrue(runOnce(this::testAutos).ignoringDisable(true));
    drivetrainAtReefTargetTrig = m_drivebase.atPose(this.offsetSelectedReefPose);
    drivetrainCloseMoveArmTrig = m_drivebase.safeToMoveArm(this.offsetSelectedReefPose);
    drivetrainSafeToAllignTrig = m_drivebase.safeToReefAlign(this.offsetSelectedReefPose);
    m_autoChooser.addRoutine("splitCheeseRoutine", this::splitPathAutoRoutine);
    // m_autoChooser.addCmd("HIJKL_SL3", this::HIJKL_SL3);

  }

  public void addAutos() {
    autos.put("IJKL_FLEX", () -> flexAuto(POI.STI, POI.SL3, POI.I, POI.J, POI.K, POI.L));
    // POI.J, POI.K, POI.L, POI.A));

    for (Entry<String, Supplier<Command>> entry : autos.entrySet()) {
      m_autoChooser.addCmd(entry.getKey(), entry.getValue());
    }
  }

  private Alert successfulAutoTest = new Alert("Successfully Checked Autos", AlertType.kInfo);

  public void testAutos() {
    for (Entry<String, Supplier<Command>> entry : autos.entrySet()) {
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
    return defer(() -> {
      var cachedTarget = target.get();
      Supplier<Pose2d> targetSup = () -> cachedTarget;
      return race(
          waitUntil(
              m_drivebase.atPose(targetSup)
                  .and(
                      () -> {
                        return m_arm.atPosition(position);
                      })), // Proxy so hand isn't directly required
          waitSeconds(4),
          m_drivebase.driveToPoseSupC(targetSup))
          .andThen(deadline(
              waitSeconds(0.25).andThen(outtake().withTimeout(outtakeSeconds).asProxy()), m_drivebase.stop()));
    }, Set.of(m_drivebase));
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
    return () -> original
        .get()
        .plus(new Transform2d(new Translation2d(0, getDistanceSensorOffset()), Rotation2d.kZero));
  }

  private POI selectedReefPOI() {
    return switch (m_board.getBranch()) {
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

  @Logged
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

  public enum ReefSide {
    R1(POI.A, POI.B, POI.R1, Arm.Positions.HIGH_ALGAE),
    R2(POI.C, POI.D, POI.R2, Arm.Positions.LOW_ALGAE),
    R3(POI.E, POI.F, POI.R3, Arm.Positions.HIGH_ALGAE),
    R4(POI.G, POI.H, POI.R4, Arm.Positions.LOW_ALGAE),
    R5(POI.I, POI.J, POI.R5, Arm.Positions.HIGH_ALGAE),
    R6(POI.K, POI.L, POI.R6, Arm.Positions.LOW_ALGAE);

    public final POI left;
    public final POI right;
    public final POI algae;
    public final ArmPosition algaeArm;

    private ReefSide(POI left, POI right, POI algae, ArmPosition algaeArm) {
      this.left = left;
      this.right = right;
      this.algae = algae;
      this.algaeArm = algaeArm;
    }
  }

  @Logged
  public ReefSide closestSide() {
    var reef = POI.REEF.flippedPose();
    var pose = m_drivebase.getPose();
    var direction = pose.relativeTo(reef).getTranslation().getAngle().getRadians();
    if (direction < -5 * Math.PI / 6 || direction > 5 * Math.PI / 6) {
      return ReefSide.R1;
    }
    if (direction >= -5 * Math.PI / 6 && direction < -3 * Math.PI / 6) {
      return ReefSide.R2;
    }
    if (direction >= -3 * Math.PI / 6 && direction < -1 * Math.PI / 6) {
      return ReefSide.R3;
    }
    if (direction >= -1 * Math.PI / 6 && direction < 1 * Math.PI / 6) {
      return ReefSide.R4;
    }
    if (direction >= 1 * Math.PI / 6 && direction < 3 * Math.PI / 6) {
      return ReefSide.R5;
    } else {
      return ReefSide.R6;
    }
  }

  private ArmPosition selectedBranch() {
    return switch (m_board.getLevel()) {
      case 0 -> Arm.Positions.L1;
      case 1 -> Arm.Positions.L2;
      case 2 -> Arm.Positions.L3;
      case 3 -> Arm.Positions.L4;
      default -> Arm.Positions.STOW;
    };
  }

  private POI selectedClimb() {
    return switch (selectedClimbNumber()) {
      case 0 -> POI.CL1;
      case 1 -> POI.CL2;
      case 2 -> POI.CL3;
      default -> POI.CL1;
    };
  }

  @Logged
  public int selectedClimbNumber() {
    var poseTranslation = m_drivebase.getPose().getTranslation();
    var CL1Dist = poseTranslation.getDistance(POI.CL1.flippedPose().getTranslation());
    var CL2Dist = poseTranslation.getDistance(POI.CL2.flippedPose().getTranslation());
    var CL3Dist = poseTranslation.getDistance(POI.CL3.flippedPose().getTranslation());
    if (CL1Dist < CL2Dist && CL1Dist < CL3Dist) {
      return 0;
    }
    if (CL2Dist < CL1Dist && CL2Dist < CL3Dist) {
      return 1;
    }
    return 2;
  }

  public Command alignToClimb() {
    return defer(() -> m_drivebase.driveToPoseSupC(selectedClimb()::flippedPose), Set.of(m_drivebase));
  }

  public POI closestIntake() {
    var pose = m_drivebase.getPose();
    if (pose.getTranslation().getDistance(POI.SL3.flippedPose().getTranslation()) < pose.getTranslation()
        .getDistance(POI.SR3.flippedPose().getTranslation())) {
      return POI.SL3;
    } else {
      return POI.SR3;
    }
  }

  private Command preMoveUntilTarget(Supplier<Pose2d> target, ArmPosition finalPosition) {
    return sequence(

        m_arm.goToPosition(finalPosition.premove())
            .until(m_drivebase.safeToMoveArm(target))
            .onlyIf(m_drivebase.safeToMoveArm(target).negate()),
        m_arm.goToPosition(finalPosition));
  }

  private Command safeToReefAlign(Supplier<Pose2d> target) {
    return sequence(

        m_drivebase.goToPosition(target)
            .until(m_drivebase.safeToReefAlign(target))
            .onlyIf(m_drivebase.safeToReefAlign(target).negate()));
  }

public boolean Autos.safeToReefAlign((Supplier<Pose2d>> target)){
  m_drivebase.safeToReefAlign(this::selectedReefPose);
  return frc.robot.Autos.safeToReefAlign();
}

  @Logged
  private Trigger drivetrainAtReefTargetTrig;
  @Logged
  private Trigger drivetrainCloseMoveArmTrig;
  @Logged
  public Trigger drivetrainSafeToAllignTrig;

  public Command autoScore() {
    var target = offsetSelectedReefPose;
    return defer(
        () -> {
          return deadline(
              waitUntil(
                  m_drivebase.atPose(target)
                      .and(
                          () -> m_arm.atPosition(selectedBranch())))
          // .andThen(outtake().withTimeout(AUTO_OUTTAKE_TIME).asProxy())
              ,
              m_drivebase.driveToPoseSupC(offsetSelectedReefPose).asProxy(),
              preMoveUntilTarget(target, selectedBranch()).asProxy()).andThen(
          // new ScheduleCommand(m_arm.goToPosition(Arm.Positions.STOW))
          ).asProxy();
        }, Set.of());

  }

  public Command autoCoralIntake(POI intake) {
    return parallel(
        m_drivebase.driveToPoseSupC(intake::flippedPose),
        new ScheduleCommand(m_arm.goToPosition(Arm.Positions.INTAKE_CORAL)),
        new ScheduleCommand(
            m_hand.inCoral().until(this::hasCoral).andThen(
                m_hand.inCoral().withTimeout(0.5))));
  }

  private double bargeTargetX() {
    final double blueX = 8;
    return AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.applyX(blueX) : blueX;
  }

  public Command alignToBarge(DoubleSupplier lateralSpeed) {
    return m_drivebase.driveToX(
        this::bargeTargetX,
        lateralSpeed,
        () -> (AllianceFlipUtil.shouldFlip() ? Rotation2d.kZero : Rotation2d.k180deg));
  }

  public boolean atBargeLine() {
    return MathUtil.isNear(bargeTargetX(), m_drivebase.getPose().getX(), 1);
  }

  public Command outtake() {
    return m_hand.outCoral().alongWith(runOnce(() -> m_coralSensor.setHasCoral(false)));
  }

  public AutoTrajectory bindScore(AutoTrajectory self, ArmPosition scoringPosition, Optional<AutoTrajectory> next) {
    var finalPoseUnflipped = self.getRawTrajectory().getFinalPose(false).get();
    var finalPoseFlipped = self.getFinalPose().get();
    System.out.println(DriverStation.getAlliance());
    System.out.println(finalPoseUnflipped);
    self.atTranslation(
       finalPoseUnflipped.getTranslation(), Units.inchesToMeters(24))
        .onTrue(print("Inside Scoring Radius"))
        .onTrue(m_arm.goToPosition(scoringPosition))
        .onTrue(
          sequence(
            alignAndDrop(
              sensorOffsetPose(() -> finalPoseFlipped), scoringPosition, AUTO_OUTTAKE_TIME
            ),
            Commands.waitSeconds(AUTO_OUTTAKE_TIME),
            new ScheduleCommand(m_arm.goToPosition(Arm.Positions.INTAKE_CORAL)),
            next.map((Function<AutoTrajectory, Command>) (nextTraj)->
              waitUntil(
                  () -> m_arm.position.elevatorLength().lt(Arm.Positions.L3.elevatorLength()))
                  .andThen(nextTraj.spawnCmd())).orElse(none())
        ));
    return self;
  }
  public Command flexAuto(POI start, POI intake, POI firstScore, POI... rest)
      throws NoSuchElementException {
    final ArmPosition scoringPosition = autoScoringPosition;
    var routine = m_autoFactory.newRoutine("JKLA_SL3");
    var toReef = start.toChecked(firstScore, routine)
        .map(this::bindAutoScorePremove)
        .get();
    
    // Set up the start->first score -> intake
    routine
        .active()
        .onTrue(
            sequence(
                toReef.resetOdometry(),
                toReef.cmd()));
      
    List<POI> scores = new LinkedList<POI>();
    scores.add(firstScore);
    scores.addAll(List.of(rest));
    List<Pair<POI,POI>> scorePairs = new LinkedList<>();
    for (int i = 0; i < scores.size()-1; i++) {
      scorePairs.add(new Pair<POI,POI>(scores.get(i), scores.get(i+1)));
    }

    System.out.println(scorePairs);
    // bind the prev->poi (toReef) followed by the poi->intake (toIntake), followed by starting the next score;
    for (Pair<POI,POI> pair : scorePairs){
      
      var toIntake = pair.getFirst().toChecked(intake, routine)
          .map(this::bindIntake)
          .get();
      bindScore(toReef, scoringPosition, Optional.of(toIntake));

      var nextToReef = intake.toChecked(pair.getSecond(), routine)
      .map(this::bindAutoScorePremove)
      .get();
      var intakeFinalPose = toIntake.getRawTrajectory().getFinalPose(false).get();
      var recieveCoral= RobotBase.isSimulation() ? sequence(waitSeconds(1), runOnce(()->m_coralSensor.setHasCoral(true))) : waitUntil(this::hasCoral);
      toIntake
      .atTranslation(
        intakeFinalPose.getTranslation(), Units.inchesToMeters(24))
          .onTrue(
              sequence(
                  deadline(
                      recieveCoral,
                      m_drivebase.driveToPoseSupC(intake::flippedPose)),
                  nextToReef.spawnCmd()
      ));
      toReef = nextToReef;
    }
    bindScore(toReef, scoringPosition, Optional.empty());
    return routine.cmd();
  }

  private final double TIME_INTAKE_TO_L4 = 1.4;
  private final double AUTO_OUTTAKE_TIME = 0.25;

  private final ArmPosition autoScoringPosition = Arm.Positions.L4;
  private AutoTrajectory bindAutoScorePremove(AutoTrajectory trajectory) {
    trajectory
        .atTime(Math.max(0, trajectory.getRawTrajectory().getTotalTime() - TIME_INTAKE_TO_L4))
        .onTrue(m_arm.goToPosition(autoScoringPosition.premove()));
    return trajectory;
  }

  private AutoTrajectory bindIntake(AutoTrajectory trajectory) {
    trajectory.atTime(0).onTrue(m_arm.goToPosition(Arm.Positions.INTAKE_CORAL));
    trajectory.atTime(1)
        .onTrue(m_hand.inCoral().until(new Trigger(this::hasCoral)).andThen(m_hand.inCoral().withTimeout(0.5)));
    return trajectory;
  }

  public Command climb() {
    return parallel(m_ClimbHookS.clamp(), m_arm.Climb(),
        sequence(waitUntil(m_arm::readyToClimb),
            m_ArmBrakeS.brake()));
  }
}
