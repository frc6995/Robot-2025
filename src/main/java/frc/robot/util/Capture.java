package frc.robot.util;

import java.util.function.Supplier;

public class Capture<T> implements Supplier<T> {
    public T inner;

    public Capture(T inner) {
      this.inner = inner;
    }

    public T get() {return inner;}
  }