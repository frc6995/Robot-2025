
package frc.robot;

import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ChoreoVariables;

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
    // Intake left, right
    SL1(false),
    SL2(false),
    SL3(false),
    SR1(false),
    SR2(false),
    SR3(false),
    // Starting for corresponding pose
    STE(false),
    STF(false),
    STG(false),
    STH(false),
    STJ(false),
    STI(false),
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
    CL3(false);

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
}