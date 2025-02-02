package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class DriverDisplay {
    private DoublePublisher matchTimeEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("/DriverDisplay/matchTime").publish();
    private DoublePublisher maxTimeEntry =
        NetworkTableInstance.getDefault().getDoubleTopic("/DriverDisplay/maxTime").publish();
    private BooleanPublisher hasCoralEntry =
        NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/hasCoral").publish();
    private BooleanSupplier hasCoralSupplier = ()->false;
    public DriverDisplay setHasCoralSupplier(BooleanSupplier hasCoralSupplier) {
        this.hasCoralSupplier = hasCoralSupplier;
        return this;
    }

    private BooleanPublisher allHomed =
        NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/allHomed").publish();
    private BooleanSupplier allHomedSupplier = ()->false;
    public DriverDisplay setAllHomedSupplier(BooleanSupplier allHomed) {
        this.allHomedSupplier = allHomed;
        return this;
    }

    private IntegerPublisher branch =
    NetworkTableInstance.getDefault().getIntegerTopic("/DriverDisplay/branch").publish();
    private IntSupplier branchSupplier = ()->0;
    public DriverDisplay setBranchSupplier(IntSupplier branch) {
        this.branchSupplier = branch;
        return this;
    }

    private IntegerPublisher level =
    NetworkTableInstance.getDefault().getIntegerTopic("/DriverDisplay/level").publish();
    private IntSupplier levelSupplier = ()->0;
    public DriverDisplay setLevelSupplier(IntSupplier level) {
        this.levelSupplier = level;
        return this;
    }

    private IntegerPublisher climb =
    NetworkTableInstance.getDefault().getIntegerTopic("/DriverDisplay/climb").publish();
    private IntSupplier climbSupplier = ()->0;
    public DriverDisplay setClimbSupplier(IntSupplier climb) {
        this.climbSupplier = climb;
        return this;
    }

    private BooleanPublisher controller0Entry =
        NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/controller0").publish();
    private BooleanPublisher controller1Entry =
        NetworkTableInstance.getDefault().getBooleanTopic("/DriverDisplay/controller1").publish();
    public DriverDisplay() {}
    
    public void update() {
        maxTimeEntry.accept(DriverStation.isAutonomous()?15:135);
        matchTimeEntry.accept(DriverStation.getMatchTime());
        
        hasCoralEntry.accept(hasCoralSupplier.getAsBoolean());

        allHomed.accept(allHomedSupplier.getAsBoolean());
        level.accept(levelSupplier.getAsInt());
        branch.accept(branchSupplier.getAsInt());
        climb.accept(climbSupplier.getAsInt());
        controller0Entry.accept(DriverStation.isJoystickConnected(0));
        controller1Entry.accept(DriverStation.isJoystickConnected(1));
    }
}
