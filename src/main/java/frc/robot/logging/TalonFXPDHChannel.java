package frc.robot.logging;

import static frc.robot.logging.PowerDistributionSim.Channel.c04_FL_Drive;
import static frc.robot.logging.PowerDistributionSim.Channel.c05_FL_Steer;
import static frc.robot.logging.PowerDistributionSim.Channel.c08_FR_Steer;
import static frc.robot.logging.PowerDistributionSim.Channel.c09_FR_Drive;
import static frc.robot.logging.PowerDistributionSim.Channel.c10_BR_Steer;
import static frc.robot.logging.PowerDistributionSim.Channel.c11_BR_Drive;
import static frc.robot.logging.PowerDistributionSim.Channel.c12_BR_Steer;
import static frc.robot.logging.PowerDistributionSim.Channel.c15_BL_Drive;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import frc.robot.logging.PowerDistributionSim.Channel;

public class TalonFXPDHChannel {

  public static Map<Channel, StatusSignal<Current>> currentSignalsCanivore = new HashMap<>();
  public static Map<Channel, StatusSignal<Current>> currentSignalsRio = new HashMap<>();

  public static void registerFD(Channel channel, StatusSignal<Current> signal) {
    if (currentSignalsRio.containsKey(channel)) {
      return;
    }
    currentSignalsCanivore.putIfAbsent(channel, signal);
  }

  public static void registerFD(Channel channel, TalonFX motor) {
    registerFD(channel, motor.getSupplyCurrent());
  }

  public static void registerRio(Channel channel, StatusSignal<Current> signal) {
    if (currentSignalsCanivore.containsKey(channel)) {
      return;
    }
    currentSignalsRio.putIfAbsent(channel, signal);
  }

  public static void registerRio(Channel channel, TalonFX motor) {
    registerRio(channel, motor.getSupplyCurrent());
  }

  public static void refresh() {
    BaseStatusSignal.refreshAll(currentSignalsCanivore.values().toArray(BaseStatusSignal[]::new));
    BaseStatusSignal.refreshAll(currentSignalsRio.values().toArray(BaseStatusSignal[]::new));
  }

  /** <CAN ID, PDH CHANNEL> */
  public static Map<Integer, Channel> channels =
      Map.of(
          11, c08_FR_Steer,
          12, c09_FR_Drive,
          13, c10_BR_Steer,
          14, c11_BR_Drive,
          15, c12_BR_Steer,
          16, c15_BL_Drive,
          17, c05_FL_Steer,
          18, c04_FL_Drive);
}
