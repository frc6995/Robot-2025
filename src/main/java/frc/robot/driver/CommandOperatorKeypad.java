package frc.robot.driver;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandOperatorKeypad {
    private GenericHID m_hid;

    private IntegerPublisher selectionEntry = NetworkTableInstance.getDefault().getIntegerTopic("/DriverDisplay/selection").publish();    
    private int selection = 0;
    public enum Button {
        kLeftGrid(1),
        kCenterGrid(2),
        kRightGrid(3),
        kHighLeft(4),
        kHighCenter(5),
        kHighRight(6),
        kMidLeft(7),
        kMidCenter(8),
        kMidRight(9),
        kLowLeft(10),
        kLowCenter(11),
        kLowRight(12),
        kStowButton(13),
        kEnterKey(14);
    
        public final int value;
    
        Button(int value) {
          this.value = value;
        }

      }
    public CommandOperatorKeypad(int port) {
        m_hid = new GenericHID(port);
        
        selectionEntry.set(0);
    }

    public Trigger action() {
        return key(Button.kStowButton);
    }
    public Trigger enter() {
        return key(Button.kEnterKey);
    }

    public Trigger key(Button key) {
        return m_hid.button(key.value, CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }
    public Trigger leftGrid() {
        return key(Button.kLeftGrid);
    }

    public Trigger centerGrid() {
        return key(Button.kCenterGrid);
    }

    public Trigger rightGrid() {
        return key(Button.kRightGrid);
    }


    public int get() {
        return selection;
    }
}

