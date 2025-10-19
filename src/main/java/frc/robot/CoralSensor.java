package frc.robot;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Logged
public class CoralSensor {

    CANrange m_backCANrange = new CANrange(CoralSensorConstants.BACK_CANRANGE_ID, Robot.m_notSwerveBus);
    CANrange m_frontCANrange = new CANrange(CoralSensorConstants.CENTER_CANRANGE_ID, Robot.m_notSwerveBus);
    CANrange m_leftCANrange = new CANrange(CoralSensorConstants.LEFT_CANRANGE_ID, Robot.m_notSwerveBus);
    CANrange m_rightCANrange = new CANrange(CoralSensorConstants.RIGHT_CANRANGE_ID, Robot.m_notSwerveBus);

    CANrangeConfiguration m_backCANrangeConfigurator = new CANrangeConfiguration();
    CANrangeConfiguration m_frontCANrangeConfigurator = new CANrangeConfiguration();
    CANrangeConfiguration m_leftCANrangeConfigurator = new CANrangeConfiguration();
    CANrangeConfiguration m_rightCANrangeConfigurator = new CANrangeConfiguration();

    private double simDistance = CoralSensorConstants.MAX_DISTANCE;

    public CoralSensor() {
        m_backCANrangeConfigurator.ProximityParams.ProximityThreshold = 0.1;
        m_frontCANrangeConfigurator.ProximityParams.ProximityThreshold = 0.05;
        m_leftCANrangeConfigurator.ProximityParams.ProximityThreshold = 0.1;
        m_rightCANrangeConfigurator.ProximityParams.ProximityThreshold = 0.1;

    }

    public boolean isValid() {
        return m_backCANrange.getIsDetected().getValue();
    }

    /*
     * private double rawDistanceMeter(){
     * 
     * if(RobotBase.isSimulation()) {
     * return simDistance;
     * }
     * return tof.getRange() / 1000.0;
     * 
     * }
     */
    public void setHasCoral(boolean hasCoral) {

        if (hasCoral) {
            simDistance = 0;
        } else {
            simDistance = CoralSensorConstants.MAX_DISTANCE;
        }
    }

    public Command setHasCoralC(boolean hasCoral) {
        return Commands.runOnce(() -> setHasCoral(hasCoral));
    }

    public class CoralSensorConstants {
        public static final int BACK_CANRANGE_ID = 55;
        public static final int RIGHT_CANRANGE_ID = 56;
        public static final int LEFT_CANRANGE_ID = 57;
        public static final int CENTER_CANRANGE_ID = 58;

        public static final double MAX_DISTANCE = 0.39;
        public static final double CENTER_DISTANCE = 0.179 - Units.inchesToMeters(0.7 - 0.6);
    }

    public boolean hasCoral() {
        if (RobotBase.isSimulation()) {
            return simDistance <= 0.1;
        } else
            return getCANrangeIsDetected(m_backCANrange);
    }

    public boolean hasFrontCoral() {
        CANrange[] sensors = { m_frontCANrange, m_leftCANrange, m_rightCANrange };
        if (RobotBase.isSimulation()) {
            return simDistance <= 0.1;
        } else
            return getCANrangeIsDetected(sensors);
    }

    public boolean getCANrangeIsDetected(CANrange sensor) {
        return sensor.getIsDetected().getValue();
    }

    public boolean getCANrangeIsDetected(CANrange[] sensors) {
        int detected = 0;
        for (var i = 0; i < sensors.length; i++) {
            if (sensors[i].getIsDetected().getValue()) {
                detected++;
            }
        }
        return (detected == sensors.length);
    }
    public boolean noHasCoral() {
        return !(m_backCANrange.getIsDetected().getValue());
    }
    public Command checkCoral() {
        return Commands.runOnce(()->hasCoral());
    }
}
