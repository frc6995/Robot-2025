package frc.robot.util;

@FunctionalInterface
public interface TriConsumer<T,U,V> {
    public  void accept(T arg0, U arg1, V arg2);
}
