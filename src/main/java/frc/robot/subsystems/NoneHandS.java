package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class NoneHandS extends Hand {

    @Override
    public Command stop() {
        return Commands.runOnce(()->{}, this);
    }

    @Override
    public Command inCoral() {
       return Commands.idle(this);
    }

    @Override
    public Command outCoral() {
        return Commands.idle(this);
        
    }

    @Override
    public Command inAlgae() {
        return Commands.idle(this);
    }

    @Override
    public Command outAlgae() {
        return Commands.idle(this);
    }
    
}
