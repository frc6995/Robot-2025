package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.defer;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.select;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

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

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
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
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.operator.OperatorBoard;
import frc.robot.subsystems.ArmBrakeS;
import frc.robot.subsystems.DriveBaseS;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.led.LightStripS;
import frc.robot.subsystems.led.TopStrip.TopStates;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Capture;

@Logged(strategy = Strategy.OPT_IN)
public class Autos {
  private final DriveBaseS m_drivebase;
  private final Arm m_arm;
  private final Hand m_hand;
  private final OperatorBoard m_board;
  private final AutoFactory m_autoFactory;
  public final AutoChooser m_autoChooser;

  public final HashMap<String, Supplier<Command>> autos = new HashMap<>();
  public final ArmBrakeS m_ArmBrakeS;
  @Logged
  public final CoralSensor m_coralSensor = new CoralSensor();

  public Autos(DriveBaseS drivebase, Arm arm, Hand hand, OperatorBoard board,
      ArmBrakeS armBrakeS, TrajectoryLogger<SwerveSample> trajlogger) {
    m_drivebase = drivebase;
    m_arm = arm;
    m_hand = hand;
    m_board = board;
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
    drivetrainSafeToAlignTrig = m_drivebase.safeToReefAlign(this.offsetSelectedReefPose);
    // // m_autoChooser.addCmd("HIJKL_SL3", this::HIJKL_SL3);

  }

  public void addAutos() {
    autos.put("1.Left 3p (Home)", () -> flexAuto(POI.STJ, POI.SL3, Optional.empty(), POI.J, POI.K, POI.L, POI.J));
    autos.put("2.Right 3p", () -> flexAuto(POI.STE, POI.SR3, Optional.empty(), POI.E, POI.D, POI.C));
    autos.put("3.CenterLeft 1p", ()->flexAuto(POI.STH, POI.SL3, Optional.empty(), POI.H));
    autos.put("4.CenterRight 1p", ()->flexAuto(POI.STG, POI.SR3, Optional.empty(), POI.G));
    autos.put("5.MoveOffLine", ()->{
      var move = new SwerveRequest.RobotCentric();
      return m_drivebase.applyRequest(()->move.withVelocityX(-1)).withTimeout(1);});

    autos.put("Left 2.5p Push", ()->flexAuto(POI.STJ, POI.SL3, Optional.of(
        (routine)->{
          var traj = routine.trajectory("K-PUSH");
          traj.atTimeBeforeEnd(1).onTrue(
            m_hand.inCoral().until(new Trigger(this::hasCoral)).andThen(m_hand.inCoral().withTimeout(0.5)));
          return traj;
        }
      ), POI.J, POI.K));
      autos.put("Wheel Rad Test", ()->m_drivebase.wheelRadiusCharacterisation(1));

      autos.put("BacksideMid", () -> flexAuto(POI.STH, POI.SL3, Optional.of(
        (routine)->{
          var traj = routine.trajectory("1");
          var push = routine.trajectory("2");
          traj.atTime(1).onTrue(m_arm.goToPosition(Arm.Positions.LOW_ALGAE));
          traj.chain(push);
          
          var toScore = routine.trajectory("3");
          push.chain(toScore);
          var moveback = routine.trajectory("4");
          toScore.done().onTrue(bargeUpAndOut()).onTrue(waitSeconds(2).andThen(moveback.spawnCmd()));

          
          var push2 = routine.trajectory("5");
          moveback.atTime(0).onTrue(waitSeconds(1).andThen(m_arm.goToPosition(Arm.Positions.HIGH_ALGAE)));

          moveback.chain(push2);

          push2.atTime(0).onTrue(waitSeconds(0.7).andThen(m_arm.goToPosition(Arm.Positions.LOW_ALGAE)));

          var toScore2 = routine.trajectory("6");
          push2.chain(toScore2);


          var moveOffLineAlgae = (routine.trajectory("7"));
          toScore2.done().onTrue(bargeUpAndOut()).onTrue(waitSeconds(2).andThen(moveOffLineAlgae.spawnCmd()));
         // toScore2.chain(moveOffLineAlgae);
          //toScore2.done().onTrue(waitSeconds(2).andThen(moveOffLineAlgae.spawnCmd()));

          return traj;
        }), POI.H));
      
    
      

    // autos.put must be before here
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
      Capture<Pose2d> targetSup = new Capture<Pose2d>(target.get());
      return 
      runOnce(()->{
        targetSup.inner = target.get();
      }).andThen(
      race(
          waitUntil(
              m_drivebase.atPose(targetSup)
                  .and(
                      () -> {
                        return m_arm.atPosition(position);
                      })), // Proxy so hand isn't directly required
          waitSeconds(4),
          m_drivebase.driveToPoseSupC(targetSup))
          .andThen(deadline(
              waitSeconds(0.25).andThen(outtake().withTimeout(outtakeSeconds).asProxy()), m_drivebase.stop())));
    
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

  public POI selectedReefPOI() {
    if (m_board.getLevel() != 0) {
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
    else {     return switch (m_board.getBranch()) {
      case 0 -> POI.L1_A;
      case 1 -> POI.L1_B;
      case 2 -> POI.L1_C;
      case 3 -> POI.L1_D;
      case 4 -> POI.L1_E;
      case 5 -> POI.L1_F;
      case 6 -> POI.L1_G;
      case 7 -> POI.L1_H;
      case 8 -> POI.L1_I;
      case 9 -> POI.L1_J;
      case 10 -> POI.L1_K;
      case 11 -> POI.L1_L;
      default -> POI.L1_A;
    };
  }
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

  public enum AlgaeHeight {
    LOW(Arm.Positions.LOW_ALGAE),
    HIGH(Arm.Positions.HIGH_ALGAE);
    AlgaeHeight(ArmPosition position) {
          this.position = position;
        }
    
        public final ArmPosition position;
    
  }
  public enum ReefSide {
    R1(POI.A, POI.B, POI.R1, AlgaeHeight.HIGH, Rotation2d.kZero),
    R2(POI.C, POI.D, POI.R2, AlgaeHeight.LOW, Rotation2d.fromDegrees(60)),
    R3(POI.E, POI.F, POI.R3, AlgaeHeight.HIGH, Rotation2d.fromDegrees(120)),
    R4(POI.G, POI.H, POI.R4, AlgaeHeight.LOW, Rotation2d.fromDegrees(180)),
    R5(POI.I, POI.J, POI.R5, AlgaeHeight.HIGH, Rotation2d.fromDegrees(240)),
    R6(POI.K, POI.L, POI.R6, AlgaeHeight.LOW, Rotation2d.fromDegrees(300));

    public final POI left;
    public final POI right;
    public final POI algae;
    public final AlgaeHeight algaeHeight;
    public final ArmPosition algaeArm;
    public final Rotation2d faceAlgaeHeading;

    private ReefSide(POI left, POI right, POI algae, AlgaeHeight height, Rotation2d allianceRelativeFaceAlgae) {
      this.left = left;
      this.right = right;
      this.algae = algae;
      this.algaeHeight = height;
      this.algaeArm = height.position;
      this.faceAlgaeHeading = allianceRelativeFaceAlgae;
    }
  }

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

  public Command intakePositionWithToggle() {
    return Commands.either(
      m_arm.goToPosition(Arm.Positions.INTAKE_CORAL).until(m_board.toggle().negate()),
      m_arm.goToPosition(Arm.Positions.WALL_INTAKE_CORAL).until(m_board.toggle()),
      
     m_board.toggle()).repeatedly();
  }

  public POI closestIntake() {
    if (onLeftHalf()) {
      return POI.SL3;
    } else {
      return POI.SR3;
    }
  }

  public Rotation2d intakeHeadingAllianceRelative() {
    return closestIntake().bluePose.getRotation();
  }

  public boolean onLeftHalf() {
    var pose = m_drivebase.getPose();
    return pose.getTranslation().getDistance(POI.SL3.flippedPose().getTranslation()) < pose.getTranslation()
        .getDistance(POI.SR3.flippedPose().getTranslation());
  }

  private Command preMoveUntilTarget(Supplier<Pose2d> target, ArmPosition finalPosition) {
    return sequence(

        m_arm.goToPosition(finalPosition.premove().safeWrist())
            .until(m_drivebase.safeToMoveArm(target))
            .onlyIf(m_drivebase.safeToMoveArm(target).negate()),
            goToPositionWristLast(finalPosition)
        );
  }
  private Command preMoveUntilReefLevel(Supplier<Pose2d> target, Supplier<Integer> level) {
    return select(Map.of(
      0, new ScheduleCommand(m_arm.goToPosition(Arm.Positions.L1)),
      1, preMoveUntilTarget(target, Arm.Positions.L2),
      2, preMoveUntilTarget(target, Arm.Positions.L3),
      3, preMoveUntilTarget(target, Arm.Positions.L4)
    ),level);
  }

  private Command goToPositionWristLast(ArmPosition finalPosition) {
    return sequence(
      m_arm.goToPosition(finalPosition.safeWrist()).until(
      ()->m_arm.getPosition().withinTolerance(finalPosition.safeWrist(),
        Degrees.of(10).in(Radians), Inches.of(24).in(Meters), Degrees.of(360).in(Radians))
    ),
    m_arm.goToPosition(finalPosition)
    );
  }


// TODO: implement safetoreefalign
/*public boolean Autos.safeToReefAlign((Supplier<Pose2d>> target)){
  m_drivebase.safeToReefAlign(this::selectedReefPose);
  return frc.robot.Autos.safeToReefAlign();
}*/


  private Trigger drivetrainAtReefTargetTrig;
  private Trigger drivetrainCloseMoveArmTrig;
  public Trigger drivetrainSafeToAlignTrig;

  public Command autoScore() {
    Capture<Pose2d> targetSup = new Capture<Pose2d>(offsetSelectedReefPose.get());
          return 
          parallel(
              m_drivebase.driveToPoseSupC(targetSup).asProxy(),
                preMoveUntilReefLevel(targetSup, m_board::getLevel)
              .asProxy())
          .beforeStarting(()->{
            targetSup.inner = offsetSelectedReefPose.get();
          })
              
              .asProxy();
  }

  public Command armToClosestAlgae(){
    return
      Commands.select(
      Map.of(
        AlgaeHeight.LOW, new ScheduleCommand(m_arm.goToPosition(Arm.Positions.LOW_ALGAE)),
        AlgaeHeight.HIGH, new ScheduleCommand(m_arm.goToPosition(Arm.Positions.HIGH_ALGAE))
      ),
      ()->closestSide().algaeHeight);
  }

  public Command autoCoralIntake() {
    return parallel(
        new ScheduleCommand(intakePositionWithToggle()),
        new ScheduleCommand(
            m_hand.inCoral().until(this::hasCoral).andThen(
                parallel(
                  new ScheduleCommand(m_arm.goToPosition(Arm.Positions.POST_INTAKE_CORAL)),
                  m_hand.inCoral().withTimeout(0.5)).andThen(
                    new ScheduleCommand(

                    LightStripS.top.stateC(()->TopStates.Intaked).withTimeout(1)
                    )
                  )
            )
        )
    );
  }

  public Command bargeUpAndOut() {
    return deadline(
      m_hand.inAlgae().until(()-> m_arm.position.elevatorMeters() > 
        Arm.Positions.SCORE_BARGE.elevatorMeters() - Units.inchesToMeters(12))
          .andThen(m_hand.outAlgae().withTimeout(0.5)),
      m_arm.goToPosition(Arm.Positions.SCORE_BARGE).asProxy()
      
    ).andThen(
      parallel(
        new ScheduleCommand(m_arm.goToPosition(Arm.Positions.STOW)),
        new ScheduleCommand(m_hand.inAlgae())
      )
    );
  }
  private double bargeTargetX() {
    final double blueX = 8.21 - Units.inchesToMeters(18);
    return AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.applyX(blueX) : blueX;
  }

  public Rotation2d bargeTargetHeading() {
    return AllianceFlipUtil.shouldFlip() ? Rotation2d.k180deg : Rotation2d.kZero;
  }
  public Command alignToBarge(DoubleSupplier lateralSpeed) {
    return m_drivebase.driveToPoseSupC(()->{
      var start = m_drivebase.getPose();
      var target = new Pose2d(bargeTargetX(), start.getY(), bargeTargetHeading());
      return target;
    });
  }

  public boolean atBargeLine() {
    return MathUtil.isNear(bargeTargetX(), m_drivebase.getPose().getX(), Units.inchesToMeters(2)) && 
      MathUtil.isNear(0, m_drivebase.getPoseHeading().minus(bargeTargetHeading()).getRadians(), Units.degreesToRadians(10));
  }

  public Command outtake() {
    return m_hand.outCoral().alongWith(runOnce(() -> m_coralSensor.setHasCoral(false)));
  }

  public AutoTrajectory bindScore(AutoTrajectory self, ArmPosition scoringPosition, Optional<AutoTrajectory> next) {
    var finalPoseUnflipped = self.getRawTrajectory().getFinalPose(false).get();
    var finalPoseFlipped = self.getFinalPose().get();
    System.out.println(DriverStation.getAlliance());
    System.out.println(finalPoseUnflipped);
    // self.atTranslation(
    //    finalPoseUnflipped.getTranslation(), Units.inchesToMeters(6))
    //   .onTrue(m_arm.goToPosition(scoringPosition))
    // self.atTranslation(finalPoseUnflipped.getTranslation(), Units.inchesToMeters(12))
    // .onTrue(
    //   goToPositionWristLast(scoringPosition)
    // );
    self.atTranslation(
       finalPoseUnflipped.getTranslation(), Units.inchesToMeters(6))
        .onTrue(print("Inside Scoring Radius"))
        .onTrue(goToPositionWristLast(scoringPosition))
        .onTrue(
          sequence(
            alignAndDrop(
              sensorOffsetPose(() -> finalPoseFlipped), scoringPosition, AUTO_OUTTAKE_TIME
            ),
            //Commands.waitSeconds(AUTO_OUTTAKE_TIME),
            new ScheduleCommand(m_arm.goToPosition(Arm.Positions.WALL_INTAKE_CORAL)),
            next.map((Function<AutoTrajectory, Command>) (nextTraj)->
              waitUntil(
                  () -> m_arm.position.elevatorLength().lt(Arm.Positions.L3.elevatorLength().plus(Inches.of(1))))
                  .andThen(nextTraj.spawnCmd())).orElse(none())
        ));
    return self;
  }
  public Command flexAuto(POI start, POI intake, Optional<Function<AutoRoutine, AutoTrajectory>> after, POI firstScore, POI... rest)
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
      var recieveCoral= RobotBase.isSimulation() ? sequence(waitSeconds(0.4), runOnce(()->m_coralSensor.setHasCoral(true))) : waitUntil(this::hasCoral);
      toIntake
      .done()
          .onTrue(
              sequence(
                  deadline(
                      recieveCoral,
                      m_drivebase.driveToPoseSupC(intake::flippedPose)),
                  nextToReef.spawnCmd()
      ));
      toReef = nextToReef;
    }
    bindScore(toReef, scoringPosition, after.map((func)->func.apply(routine)));
    return routine.cmd();
  }

  private final double TIME_INTAKE_TO_L4 = 1.4;
  private final double AUTO_OUTTAKE_TIME = 0.25;

  private final ArmPosition autoScoringPosition = Arm.Positions.L4;
  private AutoTrajectory bindAutoScorePremove(AutoTrajectory trajectory) {
    trajectory
        .atTime(Math.max(0, trajectory.getRawTrajectory().getTotalTime() - TIME_INTAKE_TO_L4))
        .onTrue(m_arm.goToPosition(autoScoringPosition.premove().safeWrist()));
    return trajectory;
  }

  private AutoTrajectory bindIntake(AutoTrajectory trajectory) {
    trajectory.atTime(1)
        .onTrue(m_hand.inCoral().until(new Trigger(this::hasCoral)).andThen(m_hand.inCoral().withTimeout(0.5)));
    return trajectory;
  }
}
