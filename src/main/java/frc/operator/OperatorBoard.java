package frc.operator;

import edu.wpi.first.epilogue.Logged;

@Logged
public abstract class OperatorBoard {
    int branch = 0;
    int climb = 0;
    int level = 0;
    public OperatorBoard(int port) {}

    public abstract int getBitfield();

    public void poll() {
        var bitfield = getBitfield();
        branch = bitfield & 0xf;
        level = (bitfield >> 4) & 0x3;
        climb = (bitfield >> 6) & 0x3;
    }
}
