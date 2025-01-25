package frc.robot.logging;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.HashMap;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
  public record Signals(
      StatusSignal<Current> torqueCurrent,
      StatusSignal<Current> statorCurrent,
      StatusSignal<Angle> position,
      StatusSignal<Voltage> voltage) {}
  ;

  public HashMap<Integer, Signals> talons = new HashMap<>();

  public TalonFXLogger() {
    super(TalonFX.class);
  }

  public void refreshAll() {
    for (Integer i : talons.keySet()) {
      var object = talons.get(i);
      BaseStatusSignal.refreshAll(
          object.statorCurrent(), object.torqueCurrent(), object.position(), object.voltage());
    }
  }

  @Override
  protected void update(EpilogueBackend dataLogger, TalonFX object) {
    var signals = talons.get(object.getDeviceID());
    if (signals == null) {
      signals =
          new Signals(
              object.getTorqueCurrent(),
              object.getStatorCurrent(),
              object.getPosition(),
              object.getMotorVoltage());
      talons.put(object.getDeviceID(), signals);
    }
    dataLogger.log("statorCurrent", signals.statorCurrent.getValue().baseUnitMagnitude());
    dataLogger.log("torqueCurrent", signals.torqueCurrent.getValue().baseUnitMagnitude());
    dataLogger.log("position", signals.position.getValue().in(Rotation));
    dataLogger.log("voltage", signals.voltage.getValueAsDouble());
  }
}
