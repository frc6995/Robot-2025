package frc.operator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged
public abstract class OperatorBoard {
  int branch = 0;
  int climb = 0;
  int level = 0;

  public OperatorBoard(int port) {}

  public abstract int getBitfield();

  public int getBranch() {
    return branch;
  }

  public abstract boolean getLeft();

  public abstract boolean getCenter();

  public abstract boolean getRight();

  public abstract boolean getToggle();

  public Trigger left() {
    return new Trigger(this::getLeft);
  }

  public Trigger center() {
    return new Trigger(this::getCenter);
  }

  public Trigger right() {
    return new Trigger(this::getRight);
  }

  public Trigger toggle() {
    return new Trigger(this::getToggle);
  }

  public int getLevel() {
    return level;
  }

  public void poll() {
    var bitfield = getBitfield();
    branch = bitfield & 0xf;
    level = (bitfield >> 4) & 0x3;
  }
}
