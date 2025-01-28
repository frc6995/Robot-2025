package frc.operator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

public class RealOperatorBoard extends OperatorBoard {
    final int port;
    public RealOperatorBoard(int port){
        super(port);
        this.port = port;
    }

    @Override
    public int getBitfield() {
        return DriverStation.getStickButtons(port);
    }
}
