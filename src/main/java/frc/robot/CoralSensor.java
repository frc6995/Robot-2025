package frc.robot;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

@Logged
public class CoralSensor {
    private final TimeOfFlight tof = new TimeOfFlight(CoralSensorConstants.CAN_ID);
    private double simDistance = CoralSensorConstants.MAX_DISTANCE;
    public CoralSensor(){
        tof.setRangingMode(RangingMode.Short, 24);
        tof.setRangeOfInterest(8,8,12,12);

    }
    public boolean isValid() {
        return tof.isRangeValid();
    }
    private double rawDistanceMeter(){
        if(RobotBase.isSimulation()) {
            return simDistance;
        }
        return tof.getRange() / 1000.0;
        
    }
    public void setHasCoral(boolean hasCoral) {
        if (hasCoral) {
            simDistance = 0;
        }
        else {
            simDistance = Units.inchesToMeters(4);
        }
    }
    public class CoralSensorConstants {
        public static final int CAN_ID = 52;
        public static final double MAX_DISTANCE = 0.39;
        public static final double CENTER_DISTANCE = 0.179 - Units.inchesToMeters(0.7-0.6);
    }

    public boolean hasCoral(){
        return rawDistanceMeter() < 0.1 && (RobotBase.isSimulation()  || (tof.getRangeSigma() < 10)) ;
    }
}
