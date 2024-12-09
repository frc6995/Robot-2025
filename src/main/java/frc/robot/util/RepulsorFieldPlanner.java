package frc.robot.util;
import java.util.ArrayList;
import java.util.EnumSet;
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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        double radius = 0.5;
        public PointObstacle(Translation2d loc, double strength, boolean positive) {
            super(strength, positive);
            this.loc = loc;
        }
        public Force getForceAtPosition(Translation2d position, Translation2d target) {
            var dist = loc.getDistance(position);
            if (dist > 4) {return new Force();}
            var outwardsMag = distToForceMag(loc.getDistance(position)-radius);
            var initial = new Force(
                outwardsMag,
                position.minus(loc).getAngle()
                );
            // theta = angle between position->target vector and obstacle->position vector
            var theta = target.minus(position).getAngle().minus(position.minus(loc).getAngle());
            double mag = outwardsMag * Math.signum(Math.sin(theta.getRadians()/2)) / 2;

            // if (theta.getRadians() > 0) {
                return initial.rotateBy(Rotation2d.kCCW_90deg).div(initial.getNorm()).times(mag).plus(initial);                
            // } else {
            //     return initial.rotateBy(Rotation2d.kCW_90deg).div(initial.getNorm()).times(mag).plus(initial);
            // }
        }
    }

    static class SnowmanObstacle extends Obstacle {
        Translation2d loc;
        double radius = 0.5;
        public SnowmanObstacle(Translation2d loc, double strength, boolean positive) {
            super(strength, positive);
            this.loc = loc;
        }
        public Force getForceAtPosition(Translation2d position, Translation2d target) {            
            var targetToLoc = loc.minus(target);
            var targetToLocAngle = targetToLoc.getAngle();
            // 1 meter away from loc, opposite target.
            var sidewaysCircle = new Translation2d(1, targetToLoc.getAngle()).plus(loc);
            var dist = loc.getDistance(position);
            var sidewaysDist = sidewaysCircle.getDistance(position);
            if (dist > 2 && sidewaysDist > 2) {return new Force();}
            var sidewaysMag = distToForceMag(sidewaysCircle.getDistance(position)) / 2;
            var outwardsMag = distToForceMag(loc.getDistance(position));
            var initial = new Force(
                outwardsMag,
                position.minus(loc).getAngle()
                );
            
            // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
            var sidewaysTheta = target.minus(position).getAngle().minus(position.minus(sidewaysCircle).getAngle());
            
            double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
            var sidewaysAngle = targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg);
            return new Force(sideways, sidewaysAngle).plus(initial);                
        }
    }

    static class HorizontalObstacle extends Obstacle {
        double y;
        public HorizontalObstacle(double y, double strength, boolean positive) {
            super(strength, positive);
            this.y = y;
        }
        public Force getForceAtPosition(Translation2d position, Translation2d target) {
            var dist = Math.abs(position.getY() -y);
            if (dist > 1) {return Force.kZero;}
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
            var dist = Math.abs(position.getX() -x);
            if (dist > 1) {return Force.kZero;}
            return new Force(
                
                distToForceMag( x-position.getX())
                , 0
                );
        }
    }

    public static final double GOAL_STRENGTH = 1;

    public static final List<Obstacle> FIELD_OBSTACLES = List.of(
    new SnowmanObstacle(new Translation2d(5.56, 2.74),  0.4, true),
    new SnowmanObstacle(new Translation2d(3.45, 4.07),  0.4, true),
    new SnowmanObstacle(new Translation2d(5.56, 5.35),  0.4, true),
    new SnowmanObstacle(new Translation2d(11.0, 2.74),  0.4, true),
    new SnowmanObstacle(new Translation2d(13.27, 4.07), 0.4, true),
    new SnowmanObstacle(new Translation2d(11.0, 5.35),  0.4, true)
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
    private final static int ARROWS_X = 40;
    private final static int ARROWS_Y = 20;
    private final static int ARROWS_SIZE = (ARROWS_X + 1) * (ARROWS_Y + 1);
    private ArrayList<Pose2d> arrows = new ArrayList<>(ARROWS_SIZE);
    public RepulsorFieldPlanner() {
        fixedObstacles.addAll(FIELD_OBSTACLES);
        fixedObstacles.addAll(WALLS);
        for (int i = 0; i < ARROWS_SIZE; i++) {
            arrows.add(new Pose2d());
        }
        var topic = NetworkTableInstance.getDefault().getBooleanTopic("useGoalInArrows");
        topic.publish().set(useGoalInArrows);
        NetworkTableListener.createListener(topic,EnumSet.of(Kind.kValueAll), (event)->{
            useGoalInArrows = event.valueData.value.getBoolean();
            updateArrows();
        });
        topic.subscribe(useGoalInArrows);
    }
    private boolean useGoalInArrows = false;
    private Pose2d arrowBackstage = new Pose2d(-10, -10, Rotation2d.kZero);
    // A grid of arrows drawn in AScope
    void updateArrows () {
        for (int x = 0; x <= ARROWS_X; x++) {
            for (int y = 0; y <= ARROWS_Y; y++) {
                var translation = new Translation2d(x*FIELD_LENGTH/ARROWS_X, y*FIELD_WIDTH/ARROWS_Y);
                var force = getObstacleForce(translation, goal().getTranslation());
                if (useGoalInArrows) {
                    force = force.plus(getGoalForce(translation,goal().getTranslation()));
                }
                if (force.getNorm() < 1e-6) {
                    arrows.set(x*(ARROWS_Y+1) + y, arrowBackstage);
                } else {
                var rotation = force.getAngle();
                
                arrows.set(x*(ARROWS_Y+1) + y, new Pose2d(translation, rotation));
                }
            }
        }
    }
    Force getGoalForce(Translation2d curLocation, Translation2d goal) {
            var displacement = goal.minus(curLocation);
            if (displacement.getNorm() == 0) {
                return new Force();
            }
            var direction = displacement.getAngle();
            var mag = GOAL_STRENGTH * (1 + 1.0/(0.0001 + displacement.getNorm()));
            return new Force(mag, direction);
    }
    Force getObstacleForce(Translation2d curLocation, Translation2d target) {
        var force = Force.kZero;
        for (Obstacle obs : fixedObstacles) {
            force = force.plus(obs.getForceAtPosition(curLocation, target));
        }
        return force;
    }
    Force getForce(Translation2d curLocation, Translation2d target) {
        var goalForce = getGoalForce(curLocation, target).plus(getObstacleForce(curLocation, target));
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
    public SwerveSample getCmd(Pose2d pose, ChassisSpeeds currentSpeeds, double maxSpeed) {
        Translation2d speedPerSec = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        double currentSpeed = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        double stepSize_m = maxSpeed * 0.02; // TODO 
        if (goalOpt.isEmpty()) {
            return sample(
                pose.getTranslation(),
                pose.getRotation(), 0,0,0);
        }
        else {
            var startTime = System.nanoTime();
            var goal = goalOpt.get();
            var curTrans = pose.getTranslation();
            var err = curTrans.minus(goal);
            if (err.getNorm() < stepSize_m * 1.5) {
                return sample(goal, pose.getRotation(), 0,0,0);
            } else {
                var obstacleForce = getObstacleForce(curTrans, goal);

                var netForce = getGoalForce(curTrans, goal).plus(obstacleForce);
                SmartDashboard.putNumber("forceLog", netForce.getNorm());
                // Calculate how quickly to move in this direction
                var closeToGoalMax = maxSpeed * Math.min(err.getNorm() / 2, 1);
                
                stepSize_m = Math.min(maxSpeed, closeToGoalMax) * 0.02;
                var step = new Translation2d(stepSize_m, netForce.getAngle());
                var intermediateGoal = curTrans.plus(step);
                var endTime = System.nanoTime();
                SmartDashboard.putNumber("repulsorTimeS", (endTime - startTime) / 1e9);
                return sample(intermediateGoal, pose.getRotation(), step.getX()/0.02, step.getY()/0.02, 0);
            }

        }
        
    }

    public ArrayList<Translation2d> getTrajectory(Translation2d current, Translation2d goalTranslation, double stepSize_m) {
        goalTranslation = goalOpt.orElse(goalTranslation);
        ArrayList<Translation2d> traj = new ArrayList<>();
        Translation2d robot = current;
        for (int i = 0; i < 400; i++){
            var err = robot.minus(goalTranslation);
            if (err.getNorm() < stepSize_m * 1.5) {
            traj.add(goalTranslation);
            break;
            } else {
                var netForce = getForce(robot, goalTranslation);
                if (netForce.getNorm() == 0) {
                    break;
                }
                var step = new Translation2d(stepSize_m, netForce.getAngle());
                var intermediateGoal = robot.plus(step);
                traj.add(intermediateGoal);
                robot = intermediateGoal;
            }
        }
        return traj;
    }
}
