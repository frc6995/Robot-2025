package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ChoreoVariables;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

public enum POI {
  REEF(false),
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
  L1_B(true),
  L1_D(L1_B, Rotation2d.fromDegrees(60)),
  L1_F(L1_B, Rotation2d.fromDegrees(120)),
  L1_H(L1_B, Rotation2d.fromDegrees(180)),
  L1_J(L1_B, Rotation2d.fromDegrees(240)),
  L1_L(L1_B, Rotation2d.fromDegrees(300)),
  L1_A(true),
  L1_C(L1_A, Rotation2d.fromDegrees(60)),
  L1_E(L1_A, Rotation2d.fromDegrees(120)),
  L1_G(L1_A, Rotation2d.fromDegrees(180)),
  L1_I(L1_A, Rotation2d.fromDegrees(240)),
  L1_K(L1_A, Rotation2d.fromDegrees(300)),
  L2_B(true),
  L2_D(L2_B, Rotation2d.fromDegrees(60)),
  L2_F(L2_B, Rotation2d.fromDegrees(120)),
  L2_H(L2_B, Rotation2d.fromDegrees(180)),
  L2_J(L2_B, Rotation2d.fromDegrees(240)),
  L2_L(L2_B, Rotation2d.fromDegrees(300)),
  L2_A(true),
  L2_C(L2_A, Rotation2d.fromDegrees(60)),
  L2_E(L2_A, Rotation2d.fromDegrees(120)),
  L2_G(L2_A, Rotation2d.fromDegrees(180)),
  L2_I(L2_A, Rotation2d.fromDegrees(240)),
  L2_K(L2_A, Rotation2d.fromDegrees(300)),
  // Intake left, right
  SL1(false),
  SL2(false),
  SL3(false),
  SR1(false),
  SR2(false),
  SR3(false),
  // Starting for corresponding pose
  STA(false),
  STAP(false),
  STAW(false),
  STC(false),
  STE(false),
  STF(false),
  STG(false),

  STH(false),
  STJ(false),
  STI(false),
  STL(false),
  STLW(false),
  STCW(false),
  // Algae lineup for intake
  R1(false),
  R2(false),
  R3(false),
  R4(false),
  R5(false),

  R6(false),
  PROC(false),
  // Climb
  CL1(false),

  CL2(false),
  CL3(false),
  LP1(false),
  LP2(false),
  LP3(false);

  public final Pose2d bluePose;
  public final Pose2d redPose;
  public final boolean isReef;

  private static List<String> TRAJECTORIES = List.of(Choreo.availableTrajectories());

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

  private POI(POI base, Rotation2d rotateAroundReef) {
    this.isReef = true;
    final Pose2d blueReef = ChoreoVariables.getPose("REEF");
    this.bluePose =
        blueReef.plus(
            base.bluePose.relativeTo(blueReef).rotateBy(rotateAroundReef).minus(Pose2d.kZero));
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

  public Optional<AutoTrajectory> toChecked(POI to, AutoRoutine routine) {
    var option = to(to, routine);
    if (option.isEmpty()) {

      throw new NoSuchElementException(String.format("Missing %s-%s", this.name(), to.name()));
    }
    return option;
  }

  public static POI selectedL1POI(int branch) {
    return switch (branch) {
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

  public static POI selectedPivotSidePOI(int branch) {
    return switch (branch) {
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

  public static POI selectedBatterySidePOI(int branch) {
    return switch (branch) {
      case 0 -> POI.L2_A;
      case 1 -> POI.L2_B;
      case 2 -> POI.L2_C;
      case 3 -> POI.L2_D;
      case 4 -> POI.L2_E;
      case 5 -> POI.L2_F;
      case 6 -> POI.L2_G;
      case 7 -> POI.L2_H;
      case 8 -> POI.L2_I;
      case 9 -> POI.L2_J;
      case 10 -> POI.L2_K;
      case 11 -> POI.L2_L;
      default -> POI.L2_A;
    };
  }
}
