package frc.robot;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged
public class CoralSensor {
    private final TimeOfFlight tof = new TimeOfFlight(CoralSensorConstants.CAN_ID);
    private double simDistance = CoralSensorConstants.MAX_DISTANCE;
    public CoralSensor(){
        tof.setRangingMode(RangingMode.Short, 500);
        tof.setRangeOfInterest(8,8,12,12);
        new Trigger(()->DriverStation.getStickButton(4,1))
            .onTrue(Commands.runOnce(()->this.setHasCoral(true)).ignoringDisable(true));
        new Trigger(()->DriverStation.getStickButton(4,2))
            .onTrue(Commands.runOnce(()->this.setHasCoral(false)).ignoringDisable(true));

    }
    public double distanceOffset(){
        return hasCoral()? Units.inchesToMeters(-0.1875)-(rawDistanceMeter() - CoralSensorConstants.CENTER_DISTANCE):0;
    }
    
    private double rawDistanceMeter(){
        if(RobotBase.isSimulation()) {
            return simDistance;
        }
        return tof.getRange() / 1000.0;
        
    }
    public void setHasCoral(boolean hasCoral) {
        if (hasCoral) {
            simDistance = CoralSensorConstants.CENTER_DISTANCE;
        }
        else {
            simDistance = CoralSensorConstants.MAX_DISTANCE;
        }
    }
    public class CoralSensorConstants {
        public static final int CAN_ID = 52;
        public static final double MAX_DISTANCE = 0.39;
        public static final double CENTER_DISTANCE = 0.179;
    }
    public boolean hasCoral(){
        return rawDistanceMeter() < CoralSensorConstants.MAX_DISTANCE - Units.inchesToMeters(3);
    }
}
