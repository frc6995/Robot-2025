package frc.robot;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.util.Units;

public class CoralSensor {
    private final TimeOfFlight tof = new TimeOfFlight(CoralSensorConstants.CAN_ID);
    public CoralSensor(){
        tof.setRangingMode(RangingMode.Short, 500);
    }
    public double distanceOffset(){
        return hasCoral()? (rawDistanceMeter() - CoralSensorConstants.CENTER_DISTANCE):0;
    }
    private double rawDistanceMeter(){
        return tof.getRange() / 1000.0;
        
    }
    public class CoralSensorConstants {
        public static final int CAN_ID = 52;
        public static final double MAX_DISTANCE = 0.3;
        public static final double CENTER_DISTANCE = 0.1;
    }
    public boolean hasCoral(){
        return rawDistanceMeter() < CoralSensorConstants.MAX_DISTANCE - Units.inchesToMeters(3);
    }
}
