package frc.robot;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;

@Logged
public class CoralSensor {

    CANrange m_backCANrange = new CANrange(CoralSensorConstants.BACK_CANRANGE_ID, Robot.m_notSwerveBus);
    CANrange m_rightCANrange = new CANrange(CoralSensorConstants.RIGHT_CANRANGE_ID, Robot.m_notSwerveBus);
    CANrange m_leftCANrange = new CANrange(CoralSensorConstants.LEFT_CANRANGE_ID, Robot.m_notSwerveBus);
    CANrange m_centerCANrange = new CANrange(CoralSensorConstants.CENTER_CANRANGE_ID, Robot.m_notSwerveBus);

    CANrangeConfiguration m_backCANrangeConfigurator = new CANrangeConfiguration();

    public enum CoralState {
        BackOnly,
        CenterOnly,
        LeftOnly,
        RightOnly,
        BackAndCenter,
        RightAndLeft,
        None
    }

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
        public static final int BACK_CANRANGE_ID = 55;
        public static final int RIGHT_CANRANGE_ID = 56;
        public static final int LEFT_CANRANGE_ID = 57;
        public static final int CENTER_CANRANGE_ID = 58;
        
        public static final double MAX_DISTANCE = 0.39;
        public static final double CENTER_DISTANCE = 0.179 - Units.inchesToMeters(0.7-0.6);
    }

    public CoralState getCoralSensorState(){
        if (m_backCANrange.getIsDetected().getValue() && m_centerCANrange.getIsDetected().getValue()) {
            return CoralState.BackAndCenter;
        }
        else if (m_leftCANrange.getIsDetected().getValue() && m_rightCANrange.getIsDetected().getValue()) {
            return CoralState.RightAndLeft;
        }
        else {
            return CoralState.None;
        }
    }

    public boolean hasCoral() {
        if (getCoralSensorState() == CoralState.None)
            return false;
        else
            return true;
    }

}
