package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class NoneHandS extends Hand {

    @Override
    public Command stop() {
        return Commands.runOnce(()->{}, this);
    }

    @Override
    public Command in() {
       return Commands.idle(this);
    }

    @Override
    public Command out() {
        return Commands.idle(this);
        
    }
    
}
