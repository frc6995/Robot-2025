package frc.robot.logging;
import static edu.wpi.first.units.Units.Rotation;

import java.util.HashMap;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
    record Signals(
        StatusSignal<Current> supplyCurrent,
        StatusSignal<Angle> position){};
    public static HashMap<Integer, Signals> talons = new HashMap<>();

    public TalonFXLogger() {
        super(TalonFX.class);
    }
    @Override
    protected void update(EpilogueBackend dataLogger, TalonFX object) {
        var signals = talons.get(object.getDeviceID());
        if (signals == null) {
            signals = new Signals(
                object.getSupplyCurrent(),
                object.getPosition());
            talons.put(object.getDeviceID(), signals);
        }
        BaseStatusSignal.refreshAll(
            signals.supplyCurrent,
            signals.position
        );
        dataLogger.log("current", signals.supplyCurrent.getValue().baseUnitMagnitude());
        dataLogger.log("position", signals.position.getValue().in(Rotation));
    }
}
