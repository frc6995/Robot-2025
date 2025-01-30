package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.BooleanSupplier;

public class AlertsUtil {
  public static Alert bind(Alert alert, BooleanSupplier event) {
    return bind(alert, event, CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public static Alert bind(Alert alert, BooleanSupplier event, EventLoop loop) {
    loop.bind(() -> alert.set(event.getAsBoolean()));
    return alert;
  }
}
