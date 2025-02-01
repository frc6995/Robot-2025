package frc.operator;

import edu.wpi.first.wpilibj.DriverStation;

public class RealOperatorBoard extends OperatorBoard {
    final int port;
    int selectedBranch = 0;
    int selectedLevel = 0;
    int selectedClimb = 0;
    public RealOperatorBoard(int port){
        super(port);
        this.port = port;
        
    }

    @Override
    public int getBitfield() {
        return DriverStation.getStickButtons(port);
    }
}
