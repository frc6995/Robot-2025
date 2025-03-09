package frc.operator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimOperatorBoard extends OperatorBoard {
    final int port;
    int selectedBranch = 0;
    int selectedLevel = 0;
    int selectedClimb = 0;
    CommandGenericHID hid;
    private Trigger isSim;
    public SimOperatorBoard(int port){
        super(port);
        this.port = port;
        isSim = new Trigger(()->DriverStation.getStickButtonCount(port) > 16);
        hid = new CommandGenericHID(port);
        for (int i = 0; i < 12; i++) {
            branchCmd(i);
        }
        climbCmd(0);
        climbCmd(1);
        climbCmd(2);
        levelCmd(0);
        levelCmd(1);
        levelCmd(2);
        levelCmd(3);
    }
 

    private void branchCmd(int branch) {
        isSim.and(hid.button(branch+1)).onTrue(Commands.runOnce(()->selectedBranch = branch).ignoringDisable(true));
    }

    private void climbCmd(int cage) {
        isSim.and(hid.button(cage+12+4+1)).onTrue(Commands.runOnce(()->selectedClimb = cage).ignoringDisable(true));
    }

    private void levelCmd(int level) {
        isSim.and(hid.button(level+12+1)).onTrue(Commands.runOnce(()->selectedLevel = level).ignoringDisable(true));
    }

    @Override
    public int getBitfield() {
        if (isSim.getAsBoolean()) {
            return (selectedBranch & 0xf) + ((selectedLevel & 0x3) << 4) + ((selectedClimb & 0x3) << 6);
        } else {
            return DriverStation.getStickButtons(port);
        }
    }

    @Override
    public boolean getLeft() {
        return DriverStation.getStickButton(port, 17);
    }

    @Override
    public boolean getCenter() {
        return DriverStation.getStickButton(port, 18);
    }

    @Override
    public boolean getRight() {
        return DriverStation.getStickButton(port, 19);
    }
}
