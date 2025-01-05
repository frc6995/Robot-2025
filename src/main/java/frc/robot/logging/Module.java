package frc.robot.logging;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;

@Logged
public record Module(TalonFX steer, TalonFX drive){};
