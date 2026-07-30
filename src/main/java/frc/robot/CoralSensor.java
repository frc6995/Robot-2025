package frc.robot;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;

@Logged
public class CoralSensor {

    CANrange m_backCANrange = new CANrange(CoralSensorConstants.BACK_CANRANGE_ID, Robot.m_notSwerveBus);
    CANrangeConfiguration m_backCANrangeConfigurator = new CANrangeConfiguration();

    public CoralSensor() {
        m_backCANrangeConfigurator.ProximityParams.ProximityThreshold = 0.1;

    }

    public boolean isValid() {
        return m_backCANrange.getIsDetected().getValue();
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
        return m_backCANrange.getIsDetected().getValue();
    }
}
