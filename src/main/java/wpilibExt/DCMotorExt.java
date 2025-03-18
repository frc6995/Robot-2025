package wpilibExt;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Ohms;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.proto.DCMotorProto;
import edu.wpi.first.math.system.plant.struct.DCMotorStruct;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Resistance;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;

public class DCMotorExt extends DCMotor {

  public static final DCMotorProto proto = new DCMotorProto();
  public static final DCMotorStruct struct = new DCMotorStruct();

  public final int numMotors;

  public DCMotorExt(
      double nominalVoltageVolts,
      double stallTorqueNewtonMeters,
      double stallCurrentAmps,
      double freeCurrentAmps,
      double freeSpeedRadPerSec,
      int numMotors) {
    super(
        nominalVoltageVolts,
        stallTorqueNewtonMeters,
        stallCurrentAmps,
        freeCurrentAmps,
        freeSpeedRadPerSec,
        numMotors);
    this.numMotors = numMotors;
  }

  public DCMotorExt(DCMotor motor, int numMotors) {
    super(
        motor.nominalVoltageVolts,
        motor.stallTorqueNewtonMeters,
        motor.stallCurrentAmps,
        motor.freeCurrentAmps,
        motor.freeSpeedRadPerSec,
        1);
    this.numMotors = numMotors;
  }

  /**
   * @return The torque when stalled at 12v.
   */
  public Torque stallTorque() {
    return NewtonMeters.of(stallTorqueNewtonMeters);
  }

  /**
   * @return Stator current draw when stalled at 12v.
   */
  public Current stallCurrent() {
    return Amps.of(stallCurrentAmps);
  }

  /**
   * @return Stator current draw under load at 12v.
   */
  public Current freeCurrent() {
    return Amps.of(freeCurrentAmps);
  }

  /**
   * @return Angular velocity under no load at 12v.
   */
  public AngularVelocity freeSpeed() {
    return RadiansPerSecond.of(freeSpeedRadPerSec);
  }

  /**
   * @return Motor internal resistance.
   */
  public Resistance internalResistance() {
    return Ohms.of(rOhms);
  }

  public Per<AngularVelocityUnit, VoltageUnit> kV() {
    return RadiansPerSecond.per(Volt).ofNative(KvRadPerSecPerVolt);
  }

  public Per<TorqueUnit, CurrentUnit> kT() {
    return NewtonMeters.per(Amp).ofNative(KtNMPerAmp);
  }

  /**
   * Calculate current drawn by motor with given speed and input voltage.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInputVolts The voltage being applied to the motor.
   * @return The current drawn by the motor
   */
  public Current getCurrent(AngularVelocity speed, Voltage voltageInput) {
    return Amps.of(super.getCurrent(speed.in(RadiansPerSecond), voltageInput.in(Volts)));
  }

  /**
   * Calculate torque produced by motor with given speed and input voltage.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInputVolts The voltage being applied to the motor.
   * @return The output torque.
   */
  public Torque getTorque(AngularVelocity speed, Voltage voltageInput) {
    return getTorque(getCurrent(speed, voltageInput));
  }

  /**
   * Calculate current drawn by motor for a given torque.
   *
   * @param torque The torque produced by the motor.
   * @return The current drawn by the motor.
   */
  public Current getCurrent(Torque torque) {
    return Amps.of(super.getCurrent(torque.in(NewtonMeters)));
  }

  /**
   * Calculate torque produced by the motor with a given current.
   *
   * @param current The current drawn by the stator.
   * @return The torque output.
   */
  public Torque getTorque(Current current) {
    return NewtonMeters.of(super.getTorque(current.in(Amps)));
  }

  /**
   * Calculate the voltage provided to the motor for a given torque and angular velocity.
   *
   * @param torque The torque produced by the motor.
   * @param speed The current angular velocity of the motor.
   * @return The voltage of the motor.
   */
  public Voltage getVoltage(Torque torque, AngularVelocity speed) {
    return Volts.of(super.getVoltage(torque.in(NewtonMeters), speed.in(RadiansPerSecond)));
  }

  /**
   * Calculate the voltage provided to the motor for a given torque and angular velocity.
   *
   * @param current The current drawn by the stator.
   * @param speed The current angular velocity of the motor.
   * @return The voltage of the motor.
   */
  public Voltage getVoltage(Current current, AngularVelocity speed) {
    return Volts.of(
        super.getVoltage(super.getTorque(current.in(Amps)), speed.in(RadiansPerSecond)));
  }

  /**
   * Calculates the angular speed produced by the motor at a given torque and input voltage.
   *
   * @param torque The torque produced by the motor.
   * @param voltageInput The voltage applied to the motor.
   * @return The angular speed of the motor.
   */
  public AngularVelocity getSpeed(Torque torque, Voltage voltageInput) {
    return RadiansPerSecond.of(super.getSpeed(torque.in(NewtonMeters), voltageInput.in(Volts)));
  }

  /**
   * Calculates the percent output of the motor at a given speed.
   *
   * @param speedRadPerSec The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @return The percentage of the motor's maximum speed at the given input voltage.
   */
  public double getSpeedPercent(double speedRadPerSec, double voltageInput) {
    return speedRadPerSec / (super.KvRadPerSecPerVolt * voltageInput);
  }

  /**
   * Calculates the percent output of the motor at a given speed.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @return The percentage of the motor's maximum speed at the given input voltage.
   */
  public double getSpeedPercent(AngularVelocity speed, Voltage voltageInput) {
    return this.getSpeedPercent(speed.in(RadiansPerSecond), voltageInput.in(Volts));
  }

  /**
   * Calculates the free current of the motor at a given speed.
   *
   * @param speedRadPerSec The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @return The free current of the motor.
   */
  public double getFreeCurrent(double speedRadPerSec, double voltageInput) {
    return freeCurrentAmps * getSpeedPercent(speedRadPerSec, voltageInput);
  }

  /**
   * Calculates the free current of the motor at a given speed.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @return The free current of the motor.
   */
  public Current getFreeCurrent(AngularVelocity speed, Voltage voltageInput) {
    return Amps.of(this.getFreeCurrent(speed.in(RadiansPerSecond), voltageInput.in(Volts)));
  }

  /**
   * Calculates the maximum power output of a motor at a given applied voltage and supply current
   * limit.
   *
   * @param voltageInput The voltage applied to the motor.
   * @param supplyLimitAmps The current limit of the power supply.
   * @return The maximum power output of the motor.
   */
  public double getMaxPower(double voltageInput, double supplyLimitAmps) {
    return voltageInput * supplyLimitAmps;
  }

  /**
   * Calculates the maximum power output of a motor at a given applied voltage and supply current
   * limit.
   *
   * @param voltageInput The voltage applied to the motor.
   * @param supplyLimit The current limit of the power supply.
   * @return The maximum power output of the motor.
   */
  public Power getMaxPower(Voltage voltageInput, Current supplyLimit) {
    return voltageInput.times(supplyLimit);
  }

  /**
   * Calculates the maximum stator current of the motor at a given speed, input voltage and current
   * limits.
   *
   * @param speedRadPerSec The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param supplyLimit The current limit of the power supply.
   * @param statorLimit The current limit of the stator.
   * @return The maximum stator current of the motor.
   */
  public double getMaxStator(
      double speedRadPerSec, double voltageInput, double supplyLimitAmps, double statorLimitAmps) {
    double vIn = Math.abs(voltageInput);
    double vVelo = Math.abs(voltageInput * getSpeedPercent(speedRadPerSec, voltageInput));
    double maxP = Math.abs(getMaxPower(voltageInput, supplyLimitAmps));
    double freeCurrent = getFreeCurrent(speedRadPerSec, voltageInput);

    double xSqr = (vVelo * vVelo) - (4.0 * rOhms * (-maxP + (vIn * freeCurrent)));

    if (xSqr < 0) {
      return statorLimitAmps;
    }

    double y = Math.abs(-vVelo + Math.sqrt(xSqr)) / (2.0 * rOhms);

    return Math.min(y, statorLimitAmps);
  }

  /**
   * Calculates the maximum stator current of the motor at a given speed, input voltage and current
   * limits.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param supplyLimit The current limit of the power supply.
   * @param statorLimit The current limit of the stator.
   * @return The maximum stator current of the motor.
   */
  public Current getMaxStator(
      AngularVelocity speed, Voltage voltageInput, Current supplyLimit, Current statorLimit) {
    return Amps.of(
        getMaxStator(
            speed.in(RadiansPerSecond),
            voltageInput.in(Volts),
            supplyLimit.in(Amps),
            statorLimit.in(Amps)));
  }

  /**
   * Calculates the stator current of the motor at a given speed, input voltage, supply current
   * limit and stator current limit.
   *
   * @param speedRadPerSec The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param supplyLimitAmps The current limit of the power supply.
   * @param statorLimitAmps The current limit of the stator.
   * @return The stator current of the motor.
   */
  public double getCurrent(
      double speedRadPerSec, double voltageInput, double supplyLimitAmps, double statorLimitAmps) {
    double normalStator = getCurrent(speedRadPerSec, voltageInput);
    double limitedStator =
        getMaxStator(speedRadPerSec, voltageInput, supplyLimitAmps, statorLimitAmps);
    return Math.min(Math.abs(normalStator), Math.abs(limitedStator)) * Math.signum(normalStator);
  }

  /**
   * Calculates the stator current of the motor at a given speed, input voltage, supply current
   * limit and stator current limit.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param supplyLimit The current limit of the power supply.
   * @param statorLimit The current limit of the stator.
   * @return The stator current of the motor.
   */
  public Current getCurrent(
      AngularVelocity speed, Voltage voltageInput, Current supplyLimit, Current statorLimit) {
    return Amps.of(
        getCurrent(
            speed.in(RadiansPerSecond),
            voltageInput.in(Volts),
            supplyLimit.in(Amps),
            statorLimit.in(Amps)));
  }

  /**
   * Calculates the supply current of the motor at a given speed, input voltage and stator current.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param statorCurrent The current drawn by the motor.
   * @return The supply current of the motor.
   */
  public Current getSupplyCurrent(
      AngularVelocity speed, Voltage voltageInput, Current statorCurrent) {
    if (MathUtil.isNear(0.0, voltageInput.in(Volts), 0.0001)) {
      return Amps.of(0.0);
    }
    Power statorPower =
        voltageInput.times(getSpeedPercent(speed, voltageInput)).times(statorCurrent);
    Power losses = getLosses(speed, voltageInput, statorCurrent);
    return statorPower.plus(losses).div(voltageInput);
  }

  /**
   * Calculates the output power of the motor at a given speed and torque.
   *
   * @param speed The current angular velocity of the motor.
   * @param torque The torque produced by the motor.
   * @return The output power of the motor.
   */
  public Power getOutputPower(AngularVelocity speed, Torque torque) {
    return Watts.of(speed.in(RadiansPerSecond) * torque.in(NewtonMeters));
  }

  /**
   * Calculates the losses of the motor at a given speed, input voltage and current.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param statorCurrent The current drawn by the motor.
   * @return The losses of the motor.
   */
  public Power getOutputPower(AngularVelocity speed, Current statorCurrent) {
    Torque torque = getTorque(statorCurrent);
    return Watts.of(speed.in(RadiansPerSecond) * torque.in(NewtonMeters));
  }

  /**
   * Calculates the losses of the motor at a given speed, input voltage and current.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param statorCurrent The current drawn by the motor.
   * @return The losses of the motor.
   */
  public Power getLosses(AngularVelocity speed, Voltage voltageInput, Current statorCurrent) {
    Power passiveLoss = getFreeCurrent(speed, voltageInput).times(voltageInput);
    Power resistiveLoss = Watts.of(rOhms * Math.pow(statorCurrent.in(Amps), 2.0));
    return passiveLoss.plus(resistiveLoss);
  }

  /**
   * Calculates the efficiency of the motor at a given speed, input voltage and current.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param statorCurrent The current drawn by the motor.
   * @return The efficiency of the motor.
   */
  public double getEfficiency(AngularVelocity speed, Voltage voltageInput, Current statorCurrent) {
    Power output = getOutputPower(speed, statorCurrent);
    Power losses = getLosses(speed, voltageInput, statorCurrent);
    return output.in(Watts) / (output.in(Watts) + losses.in(Watts));
  }
}
