// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.wpilibj.LEDPattern.rainbow;
import static edu.wpi.first.wpilibj.LEDPattern.solid;

import java.util.Set;
import java.util.TreeSet;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class OuterStrip {

    /**Enumerator of states for the Outer LED strip. States higher in the list have priority */
    public enum OuterStates {
        AB(solid(Color.kIndigo).atBrightness(Value.of(1))),
        CD(solid(Color.kBlue).atBrightness(Value.of(1))),
        EF(solid(Color.kGreen).atBrightness(Value.of(1))),
        GH(solid(Color.kYellow).atBrightness(Value.of(1))),
        IJ(solid(Color.kOrange).atBrightness(Value.of(1))),
        KL(solid(Color.kRed).atBrightness(Value.of(1))),
        Climbing(rainbow(255, 255).scrollAtAbsoluteSpeed(InchesPerSecond.of(10), Inches.of(0.6))),
        BlueAlliance(solid(Color.kBlue).atBrightness(Value.of(1))),
        RedAlliance(solid(Color.kRed).atBrightness(Value.of(1))),
        Default(solid(Color.kGreen).atBrightness(Value.of(1)).breathe(Seconds.of(1.6995)));
    
        public LEDPattern applier;
    
        private OuterStates(LEDPattern applier) {
            this.applier = applier;
        }
    }
    public AddressableLEDBufferView led;
    // public AddressableLEDBufferView in_right_back;
    // public AddressableLEDBufferView in_right_side;
    // public AddressableLEDBufferView in_center;
    // public AddressableLEDBufferView in_left_side;
    // public AddressableLEDBufferView in_left_back;
    // public AddressableLEDBufferView out_left_side;
    // public AddressableLEDBufferView out_left_corner;
    // public AddressableLEDBufferView out_front;
    // public AddressableLEDBufferView out_right_corner;
    private TreeSet<OuterStates> m_states = new TreeSet<>();

    /**
     * Requests the current state of the robot, determines whether the requested
     * state is a higher
     * priority than the current state, sets the current state to the requested
     * state
     *
     * @param state The requested state of the robot when the method is called
     */
    public void requestState(OuterStates state) {
        m_states.add(state);
    }

    /**
     * 
     * @param state The requested state of the outer led strip when the method is called
     * @return a run Command that calls the {@code requestState()} method
     */
    public Command stateC(Supplier<OuterStates> state) {
        return Commands.run(() -> requestState(state.get())).ignoringDisable(true);
    }
    private LEDPattern safeToReefAlignPattern = (reader, writer) -> {
        reader.forEach((i, r, g, b)->{
            if (i % 2 == 0) {
                writer.setRGB(i, 32, 32, 32);
            } else {

                writer.setRGB(i, 0, 0, 0);
            }
        });
    };

    /**
     * Periodically checks the current state of the robot and sets the LEDs to the
     * corresponding light
     * pattern
     */
    public void periodic() {
        requestState(OuterStates.Default);
        // if (DriverStation.isDisabled()) {
        // requestState(States.Disabled);
        // }
        OuterStates state = m_states.first();
        // spark.set(m_states.first().lightSpeed);
        
        var pattern = state.applier;
        if (safeToReefAlign) {
            pattern = safeToReefAlignPattern.overlayOn(pattern);
        }
        pattern.applyTo(led);
        // Do other things with the buffer

        safeToReefAlign = false;
        m_states.removeAll(Set.of(OuterStates.values()));
    }
    private boolean safeToReefAlign = false;
    public void requestSafeToAlign() {
        safeToReefAlign = true;
    }
    public OuterStrip(AddressableLEDBufferView view) {
        led = view;
    }
}