package frc.robot.logging;

import java.util.Arrays;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.PDPSim;

public class PowerDistributionSim {
    public enum Channel{
        c00,
        c01,
        c02,
        c03,
        c04_FL_Drive,
        c05_FL_Steer,
        c06,
        c07,
        c08_FR_Steer,
        c09_FR_Drive,
        c10_BR_Steer,
        c11_BR_Drive,
        c12_BR_Steer,
        c13,
        c14,
        c15_BL_Drive,
        c16,
        c17,
        c18,
        c19,
        c20,
        c21,
        c22,
        c23_Switchable;
        
        public int channel() {
            return this.ordinal();
        }
    }
    public static String[] CHANNEL_LOG_NAMES = Arrays.stream(Channel.values()).map((c)->c.toString()).toArray(String[]::new);
    public PDPSim m_sim = new PDPSim(new edu.wpi.first.wpilibj.PowerDistribution(1, ModuleType.kRev));
    public void setChannelCurrent(Channel channel, double current) {
        setChannelCurrent(channel.channel(), current);
    }
    public void setChannelCurrent(int channel, double current) {
        m_sim.setCurrent(channel, current);
    }
    // public static PowerDistributionSim instance = new PowerDistributionSim();
}
