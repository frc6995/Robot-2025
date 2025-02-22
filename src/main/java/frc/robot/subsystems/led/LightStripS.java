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



public class LightStripS {

    private static final int OUTER_STRIP_LENGTH = 138;
    private static final int TOP_STRIP_LENGTH = 192 - 138+6;
    private static AddressableLED led = new AddressableLED(5);
    private static AddressableLEDBuffer buffer = new AddressableLEDBuffer(OUTER_STRIP_LENGTH + TOP_STRIP_LENGTH);
    public static final TopStrip top;
    public static final OuterStrip outer;
    static {
        led.setLength(buffer.getLength());

        led.setData(buffer);
        led.start();
        top = new TopStrip(buffer.createView(0, TOP_STRIP_LENGTH -1));
        outer = new OuterStrip(buffer.createView(TOP_STRIP_LENGTH, TOP_STRIP_LENGTH+OUTER_STRIP_LENGTH - 1));
    }
    public static void periodic() {
        top.periodic();
        outer.periodic();
        led.setData(buffer);
    }    

    // /**
    //  * Different states of the robot, states placed higher in the list have higher
    //  * priority
    //  */
    // public static enum States {
    //     CoastMode(solid(Color.kBlue).atBrightness(Value.of(0.25))),

    //     SetupDone(solid(Color.kGreen).atBrightness(Value.of(0.25))), // set in robotPeriodic
    //     Disabled(solid(Color.kRed).atBrightness(Value.of(0.25))), // set in robotPeriodic
    //     Error(solid(Color.kRed).blink(Seconds.of(0.125))),
    //     LeftThird(leftThird(64, 0, 0)),
    //     CenterThird(centerThird(64, 64, 64)),
    //     RightThird(rightThird(64, 0, 0)),
    //     Climbing(rainbow(255, 255)),
    //     HasNote(solid(new Color(245, 224, 66))),
    //     IntakedNote(solid(new Color(245, 224, 66)).blink(Seconds.of(0.125))),
    //     Scoring(solid(Color.kBlue)),
    //     AutoAlign(rainbow(255, 255)),
    //     Passing(solid(Color.kYellow).atBrightness(Value.of(0.25))),
    //     Default(
    //             solid(Color.kGreen).atBrightness(Value.of(0.25)));
    //     // Default(setColor(0, 255, 0));

    //     public final LEDPattern setter;

    //     private States(LEDPattern setter) {
    //         this.setter = setter;
    //     }
    // }

    // // currentStates = {Disabled, Climbing, EjectingWrongColor, Intaking, Shooting,
    // // Default}

    // private static LEDPattern leftThird(int r, int g, int b) {
    //     Color color = new Color(r, g, b);
    //     LEDPattern pattern = LEDPattern.steps(Map.of(0, color, 1.0 / 3, Color.kBlack));
    //     return pattern;
    // }

    // private static LEDPattern centerThird(int r, int g, int b) {
    //     Color color = new Color(r, g, b);
    //     LEDPattern pattern = LEDPattern.steps(
    //             Map.of(0, Color.kBlack, 1.0 / 3, color, 2.0 / 3, Color.kBlack));
    //     return pattern;
    // }

    // private static LEDPattern rightThird(int r, int g, int b) {

    //     Color color = new Color(r, g, b);
    //     LEDPattern pattern = LEDPattern.steps(
    //             Map.of(0, Color.kBlack, 1. / 3, Color.kBlack, 2. / 3, color));
    //     return pattern;
    // }

}
