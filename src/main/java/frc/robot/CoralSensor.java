package frc.robot;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;

@Logged
public class CoralSensor {

    CANrange m_backCANrange = new CANrange(CoralSensorConstants.BACK_CANRANGE_ID, "CANivore");
    CANrangeConfiguration m_backCANrangeConfigurator = new CANrangeConfiguration();
//todo: set can bus
   // private final TimeOfFlight tof = new TimeOfFlight(CoralSensorConstants.CAN_ID);
    private double simDistance = CoralSensorConstants.MAX_DISTANCE;
    public CoralSensor(){
        m_backCANrangeConfigurator.ProximityParams.ProximityThreshold = 0.1;

    }
    public boolean isValid() {
        return m_backCANrange.getIsDetected().getValue();
    }
    /* 
    private double rawDistanceMeter(){
        
        if(RobotBase.isSimulation()) {
            return simDistance;
        }
        return tof.getRange() / 1000.0;
        
    
        
    }
    */
    public void setHasCoral(boolean hasCoral) {
       
       
       /*  if (hasCoral) {
            simDistance = 0;
        }
        else {
            simDistance = Units.inchesToMeters(4);
        }
            */

    }
    public class CoralSensorConstants {
        public static final int BACK_CANRANGE_ID = 54;
        public static final double MAX_DISTANCE = 0.39;
        public static final double CENTER_DISTANCE = 0.179 - Units.inchesToMeters(0.7-0.6);
    }

    public boolean hasCoral(){
        return m_backCANrange.getIsDetected().getValue();
    }
}
