// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import static edu.wpi.first.wpilibj.LEDPattern.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
public class TopStrip {
    public enum TopStates {
        Default(
                solid(Color.kBlue).atBrightness(Value.of(0.25)));
    
        public LEDPattern applier;
    
        private TopStates(LEDPattern applier) {
            this.applier = applier;
        }
    }
    public AddressableLEDBufferView led;

    private TreeSet<TopStates> m_states = new TreeSet<>();

    /**
     * Requests the current state of the robot, determines whether the requested
     * state is a higher
     * priority than the current state, sets the current state to the requested
     * state
     *
     * @param state The requested state of the robot when the method is called
     */
    public void requestState(TopStates state) {
        m_states.add(state);
    }

    public Command stateC(Supplier<TopStates> state) {
        return Commands.run(() -> requestState(state.get())).ignoringDisable(true);
    }

    /**
     * Periodically checks the current state of the robot and sets the LEDs to the
     * corresponding light
     * pattern
     */
    public void periodic() {
        requestState(TopStates.Default);
        // if (DriverStation.isDisabled()) {
        // requestState(States.Disabled);
        // }
        TopStates state = m_states.first();
        // spark.set(m_states.first().lightSpeed);
        state.applier.applyTo(led);
        // Do other things with the buffer
        m_states.removeAll(Set.of(TopStates.values()));
    }

    public TopStrip(AddressableLEDBufferView view) {
        led = view;
    }
}