package frc.robot.util;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

@Logged
public class RepulsorFieldPlanner {

    static abstract class Obstacle {
        double strength = 1.0;
        boolean positive = true;
        public Obstacle(double strength, boolean positive) {
            this.strength = strength;
            this.positive = positive;
        }
        public abstract Force getForceAtPosition(Translation2d position, Translation2d target);
        protected double distToForceMag(double dist) {
            var forceMag = strength / (0.00001 + Math.abs(dist*dist));
            forceMag *= positive ? 1 : -1;
            return forceMag;
        }

    }
    static class PointObstacle extends Obstacle {
        Translation2d loc;
        public PointObstacle(Translation2d loc, double strength, boolean positive) {
            super(strength, positive);
            this.loc = loc;
        }
        public Force getForceAtPosition(Translation2d position, Translation2d target) {
            var initial = new Force(
                distToForceMag(loc.getDistance(position)),
                position.minus(loc).getAngle()
                );
            // theta = angle between position->target vector and obstacle->position vector
            var theta = target.minus(position).getAngle().minus(position.minus(loc).getAngle());
            double mag = strength * Math.cos(theta.getRadians()/2) / Math.pow(position.getDistance(loc), 2);

            if (theta.getRadians() > 0) {
                return initial.rotateBy(Rotation2d.kCCW_90deg).div(2).plus(initial);                
            } else {
                return initial.rotateBy(Rotation2d.kCW_90deg).div(2).plus(initial);
            }
        }
    }

    static class HorizontalObstacle extends Obstacle {
        double y;
        public HorizontalObstacle(double y, double strength, boolean positive) {
            super(strength, positive);
            this.y = y;
        }
        public Force getForceAtPosition(Translation2d position, Translation2d target) {
            return new Force(
                0,
                distToForceMag(y-position.getY())                
                );
        }
    }
    static class VerticalObstacle extends Obstacle {
        double x;
        public VerticalObstacle(double x, double strength, boolean positive) {
            super(strength, positive);
            this.x = x;
        }
        public Force getForceAtPosition(Translation2d position, Translation2d target) {
            return new Force(
                
                distToForceMag( x-position.getX())
                , 0
                );
        }
    }

    public static final double GOAL_STRENGTH = 1;

    public static final List<Obstacle> FIELD_OBSTACLES = List.of(
    new PointObstacle(new Translation2d(5.56, 2.74),  0.7, true),
    new PointObstacle(new Translation2d(3.45, 4.07),  0.7, true),
    new PointObstacle(new Translation2d(5.56, 5.35),  0.7, true),
    new PointObstacle(new Translation2d(11.0, 2.74),  0.7, true),
    new PointObstacle(new Translation2d(13.27, 4.07), 0.7, true),
    new PointObstacle(new Translation2d(11.0, 5.35),  0.7, true)
    );
    static final double FIELD_LENGTH = 16.42;
    static final double FIELD_WIDTH = 8.16;
    public static final List<Obstacle> WALLS = List.of(
    new HorizontalObstacle(0.0,         0.5, true),
    new HorizontalObstacle(FIELD_WIDTH,   0.5, false),
    new VerticalObstacle(0.0,           0.5, true),
    new VerticalObstacle(FIELD_LENGTH,    0.5, false)
    );

    private List<Obstacle> fixedObstacles = new ArrayList<>();
    private Optional<Translation2d> goalOpt = Optional.empty();
    public Pose2d goal() {
        return new Pose2d(goalOpt.orElse(Translation2d.kZero), Rotation2d.kZero);
    }
    private Pose2d[] arrows = new Pose2d[200];
    public RepulsorFieldPlanner() {
        fixedObstacles.addAll(FIELD_OBSTACLES);
        fixedObstacles.addAll(WALLS);
    }
    // A grid of arrows drawn in AScope
    void updateArrows () {
        for (int x = 0; x < 20; x++) {
            for (int y = 0; y < 10; y++) {
                var translation = new Translation2d(x*FIELD_LENGTH/20, y*FIELD_WIDTH/10);
                var rotation = getForce(translation, goal().getTranslation()).getAngle();
                arrows[x*10 + y] = new Pose2d(translation, rotation);
            }
        }
    }
    Force getGoalForce(Translation2d curLocation) {
        return goalOpt.map((goal)->{
            var displacement = goal.minus(curLocation);
            if (displacement.getNorm() == 0) {
                return new Force();
            }
            var direction = displacement.getAngle();
            var mag = GOAL_STRENGTH * (1 + 1.0/(0.0001 + displacement.getNorm()));
            return new Force(mag, direction);
        }).orElse(new Force());
    }

    Force getForce(Translation2d curLocation, Translation2d target) {
        var goalForce = getGoalForce(curLocation);
        for (Obstacle obs : fixedObstacles) {
            goalForce = goalForce.plus(obs.getForceAtPosition(curLocation, target));
        }
        return goalForce;
    }

    private SwerveSample sample(Translation2d trans, Rotation2d rot, double vx, double vy, double omega) {
        return new SwerveSample(0, 
        trans.getX(), 
        trans.getY(), 
        rot.getRadians(),
        vx, vy, omega,
        0,0,0,
        new double[4],
        new double[4]
        );
    }
    public void setGoal(Translation2d goal) {
        this.goalOpt = Optional.of(goal);
        updateArrows();
    }
    public void clearGoal() {
        this.goalOpt = Optional.empty();
    }
    public SwerveSample getCmd(Supplier<Pose2d> curPose, double stepSize_m) {
        var pose = curPose.get();
        if (goalOpt.isEmpty()) {
            return sample(
                pose.getTranslation(),
                pose.getRotation(), 0,0,0);
        }
        else {
            var goal = goalOpt.get();
            var curTrans = pose.getTranslation();
            var err = curTrans.minus(goal);
            if (err.getNorm() < stepSize_m * 1.5) {
                return sample(goal, pose.getRotation(), 0,0,0);
            } else {
                var netForce = getForce(curTrans, goal);
                var step = new Translation2d(stepSize_m, netForce.getAngle());
                var intermediateGoal = curTrans.plus(step);
                return sample(intermediateGoal, pose.getRotation(), step.getX()/0.02, step.getY()/0.02, 0);
            }
        }
    }
}
