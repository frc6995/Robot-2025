package frc.robot.util;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.LocalADStar;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
@Logged
public class AStarFieldPlanner {
    private LocalADStar pathfinder = new LocalADStar();
    private PathConstraints constraints = new PathConstraints(4, 5, 6, 6);
    public ArrayList<Translation2d> poses = new ArrayList<>();
    public boolean newPathAvailable = false;
    public AStarFieldPlanner() {
        
    }
    public double pathLength = 0;
    public Command getCmd(Supplier<Translation2d> location, Supplier<Translation2d> goal) {
        AtomicReference<Translation2d> goalCache = new AtomicReference<Translation2d>(Translation2d.kZero);
        return Commands.runOnce(()->{
            goalCache.set(goal.get());
        pathfinder.setGoalPosition(goal.get());
        pathfinder.setStartPosition(location.get());
        }).andThen(Commands.waitUntil(pathfinder::isNewPathAvailable)).andThen(
            Commands.runOnce(()->{
        
        newPathAvailable = pathfinder.isNewPathAvailable();
        if (newPathAvailable) {
            pathLength = 0;
        var path = pathfinder.getCurrentPath(constraints, new GoalEndState(0, Rotation2d.kZero));
        
        if (path != null) {
            Translation2d prev = path.getAllPathPoints().get(0).position;
            poses.clear();
            for (PathPoint p : path.getAllPathPoints()) {
                poses.add(p.position);  
                pathLength += p.position.getDistance(prev);
                prev = p.position;
              }  }
        }
            })); 
    }
}
