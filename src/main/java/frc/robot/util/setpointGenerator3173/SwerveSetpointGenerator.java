package frc.robot.util.setpointGenerator3173;

import static frc.robot.util.setpointGenerator3173.Util.angleOf;
import static frc.robot.util.setpointGenerator3173.Util.angularDifference;
import static frc.robot.util.setpointGenerator3173.Util.epsilonEquals;
import static frc.robot.util.setpointGenerator3173.Util.findDriveMaxS;
import static frc.robot.util.setpointGenerator3173.Util.findSteeringMaxS;
import static frc.robot.util.setpointGenerator3173.Util.rotateBy;
import static frc.robot.util.setpointGenerator3173.Util.shouldFlipHeading;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.setpointGenerator3173.Types.ChassisConstraints;
import frc.robot.util.setpointGenerator3173.Util.LocalVars;
import wpilibExt.DCMotorExt;
import wpilibExt.MutTranslation2d;
import wpilibExt.Speeds;
import wpilibExt.Speeds.RobotSpeeds;

/**
 * Swerve setpoint generator that has been passed around so many times its hard to keep track, just
 * know i didn't write most the logic in this code that credit goes to 254 and mjansen
 *
 * <p>Takes a prior setpoint, a desired setpoint, and outputs a new setpoint that respects all the
 * kinematic constraints on module rotation and wheel velocity/torque, as well as preventing any
 * forces acting on a module's wheel from exceeding the force of friction.
 */
public class SwerveSetpointGenerator {
  private static final ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds();

  private static final int NUM_MODULES = 4;

  private final SwerveDriveKinematics kinematics;
  private final Translation2d[] moduleLocations;
  private final DCMotorExt driveMotor;
  private final double driveStatorCurrentLimitAmps,
      driveSupplyCurrentLimitAmps,
      maxDriveVelocityMPS,
      maxSteerVelocityRadsPerSec,
      massKg,
      moiKgMetersSquared,
      wheelRadiusMeters,
      wheelFrictionForce,
      torqueLoss;

  public SwerveSetpointGenerator(
      final Translation2d[] moduleLocations,
      final DCMotorExt driveMotor,
      final DCMotor angleMotor,
      final double driveStatorCurrentLimitAmps,
      final double driveSupplyCurrentLimitAmps,
      final double massKg,
      final double moiKgMetersSquared,
      final double wheelDiameterMeters,
      final double wheelCoF,
      final double torqueLoss) {

    if (moduleLocations.length != NUM_MODULES) {
      throw new IllegalArgumentException("Module locations must have 4 elements");
    }
    this.driveMotor = driveMotor;
    this.driveStatorCurrentLimitAmps = driveStatorCurrentLimitAmps;
    this.driveSupplyCurrentLimitAmps = driveSupplyCurrentLimitAmps;
    this.maxSteerVelocityRadsPerSec = angleMotor.freeSpeedRadPerSec;
    kinematics = new SwerveDriveKinematics(moduleLocations);
    this.moduleLocations = moduleLocations;
    this.massKg = massKg;
    this.moiKgMetersSquared = moiKgMetersSquared;
    this.wheelRadiusMeters = wheelDiameterMeters / 2.0;
    this.maxDriveVelocityMPS = driveMotor.freeSpeedRadPerSec * wheelRadiusMeters;

    wheelFrictionForce = wheelCoF * ((massKg / 4.0) * 9.81);
    // maxTorqueFriction = this.wheelFrictionForce * wheelRadiusMeters;
    this.torqueLoss = torqueLoss;
  }

  // a lot of work was done to reduce allocations in this hot loop,
  // migrating everything over to a vars object that gets reused
  // was the best way to do this.
  private final LocalVars VARS_TEMPLATE = new LocalVars();

  /**
   * Generate a new setpoint. Note: Do not discretize ChassisSpeeds passed into or returned from
   * this method. This method will discretize the speeds for you.
   *
   * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
   *     iteration setpoint instead of the actual measured/estimated kinematic state.
   * @param desiredRobotRelativeSpeeds The desired state of motion, such as from the driver sticks
   *     or a path following algorithm.
   * @param dt The loop time.
   * @return A Setpoint object that satisfies all the kinematic/friction limits while converging to
   *     desiredState quickly.
   */
  public SwerveSetpoint generateSetpoint(
      final SwerveSetpoint prevSetpoint,
      ChassisSpeeds desiredRobotRelativeSpeeds,
      Optional<ChassisConstraints> constraintsOpt,
      double dt) {
    double inputVoltage = RobotController.getBatteryVoltage();
    if (Double.isNaN(inputVoltage)) {
      inputVoltage = 12.0;
    } else {
      inputVoltage = Math.max(inputVoltage, RobotController.getBrownoutVoltage());
    }
    double maxSpeed = maxDriveVelocityMPS * Math.min(1, inputVoltage / 12);

    // Limit the max velocities in desired state based on constraints
    if (constraintsOpt.isPresent()) {
      ChassisConstraints constraints = constraintsOpt.get();
      maxSpeed = Math.min(maxSpeed, constraints.translation().maxVelocity());
      double linearVel =
          Math.hypot(
              desiredRobotRelativeSpeeds.vxMetersPerSecond,
              desiredRobotRelativeSpeeds.vyMetersPerSecond);
      if (linearVel > constraints.translation().maxVelocity()) {
        double mult = constraints.translation().maxVelocity() / linearVel;
        desiredRobotRelativeSpeeds.vxMetersPerSecond *= mult;
        desiredRobotRelativeSpeeds.vyMetersPerSecond *= mult;
      }
      desiredRobotRelativeSpeeds.omegaRadiansPerSecond =
          MathUtil.clamp(
              desiredRobotRelativeSpeeds.omegaRadiansPerSecond,
              -constraints.rotation().maxVelocity(),
              constraints.rotation().maxVelocity());
    }

    // https://github.com/wpilibsuite/allwpilib/issues/7332
    SwerveModuleState[] desiredModuleStates =
        kinematics.toSwerveModuleStates(desiredRobotRelativeSpeeds);
    // Make sure desiredState respects velocity limits.
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, maxSpeed);
    desiredRobotRelativeSpeeds = kinematics.toChassisSpeeds(desiredModuleStates);

    final LocalVars vars = VARS_TEMPLATE.reset();
    vars.dt = dt;
    vars.prevSpeeds = prevSetpoint.speeds().toWpilib();
    vars.desiredSpeeds = desiredRobotRelativeSpeeds;
    vars.desiredModuleStates = desiredModuleStates;
    vars.prevModuleStates = prevSetpoint.moduleStates();
    vars.minS = 1.0;
    vars.constraintsOpt = constraintsOpt;
    vars.inputVoltage = inputVoltage;

    // //logger.log("beginningVars", vars, LocalVars.struct);

    checkNeedToSteer(vars);
    //logger.log("postCheckNeedToSteer", vars, LocalVars.struct);
    makeVectors(vars);
    //logger.log("pastMakeVectors", vars, LocalVars.struct);

    solveSteering(vars);
    //logger.log("postSolveSteering", vars, LocalVars.struct);

    solveDriving(vars);
    //logger.log("postSolveDriving", vars, LocalVars.struct);

    ChassisSpeeds retSpeeds =
        ChassisSpeeds.discretize(
            new ChassisSpeeds(
                vars.prevSpeeds.vxMetersPerSecond + vars.minS * vars.dx,
                vars.prevSpeeds.vyMetersPerSecond + vars.minS * vars.dy,
                vars.prevSpeeds.omegaRadiansPerSecond + vars.minS * vars.dtheta),
            dt);

    final SwerveModuleState[] retStates = kinematics.toSwerveModuleStates(retSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(retStates, maxDriveVelocityMPS);

    AdvancedSwerveModuleState[] outputStates = new AdvancedSwerveModuleState[NUM_MODULES];
    for (int m = 0; m < NUM_MODULES; m++) {
      if (vars.steeringOverride[m] != null) {
        if (shouldFlipHeading(
            retStates[m].angle.unaryMinus().rotateBy(vars.steeringOverride[m]).getRadians())) {
          retStates[m].speedMetersPerSecond *= -1.0;
        }
        retStates[m].angle = vars.steeringOverride[m];
      }
      final double deltaRotation =
          angularDifference(
              vars.prevModuleStates[m].angle.getRadians(), retStates[m].angle.getRadians());
      if (shouldFlipHeading(deltaRotation)) {
        retStates[m].angle = retStates[m].angle.rotateBy(Rotation2d.k180deg);
        retStates[m].speedMetersPerSecond *= -1.0;
      }
      final double deltaVelocity =
          retStates[m].speedMetersPerSecond - vars.prevModuleStates[m].speedMetersPerSecond;
      outputStates[m] =
          new AdvancedSwerveModuleState(
              retStates[m].speedMetersPerSecond, retStates[m].angle, deltaVelocity / dt);
    }

    //logger.log("finalVars", vars, LocalVars.struct);

    return new SwerveSetpoint(Speeds.fromRobotRelative(retSpeeds), outputStates);
  }

  public SwerveSetpoint generateSimpleSetpoint(
      final SwerveSetpoint prevSetpoint, RobotSpeeds desiredRobotRelativeSpeeds, double dt) {
    AdvancedSwerveModuleState[] outputStates = new AdvancedSwerveModuleState[NUM_MODULES];
    SwerveModuleState[] desiredModuleStates =
        kinematics.toSwerveModuleStates(desiredRobotRelativeSpeeds.toWpilib());
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, maxDriveVelocityMPS);
    for (int m = 0; m < NUM_MODULES; m++) {
      desiredModuleStates[m].optimize(prevSetpoint.moduleStates()[m].angle);
      outputStates[m] = AdvancedSwerveModuleState.fromBase(desiredModuleStates[m]);
    }

    return new SwerveSetpoint(
        Speeds.fromRobotRelative(kinematics.toChassisSpeeds(desiredModuleStates)), outputStates);
  }

  private static void checkNeedToSteer(LocalVars vars) {
    if (epsilonEquals(vars.desiredSpeeds, ZERO_SPEEDS)) {
      vars.needToSteer = false;
      for (int m = 0; m < NUM_MODULES; m++) {
        vars.desiredModuleStates[m].angle = vars.prevModuleStates[m].angle;
        vars.desiredModuleStates[m].speedMetersPerSecond = 0.0;
      }
    }
  }

  private static void makeVectors(LocalVars vars) {
    for (int m = 0; m < NUM_MODULES; m++) {
      vars.prev[m].applyModuleState(vars.prevModuleStates[m]);
      vars.desired[m].applyModuleState(vars.desiredModuleStates[m]);
    }

    vars.dx = vars.desiredSpeeds.vxMetersPerSecond - vars.prevSpeeds.vxMetersPerSecond;
    vars.dy = vars.desiredSpeeds.vyMetersPerSecond - vars.prevSpeeds.vyMetersPerSecond;
    vars.dtheta = vars.desiredSpeeds.omegaRadiansPerSecond - vars.prevSpeeds.omegaRadiansPerSecond;
  }

  private void solveSteering(LocalVars vars) {
    // In cases where an individual module is stopped, we want to remember the right steering angle
    // to command (since inverse kinematics doesn't care about angle, we can be opportunistically
    // lazy).
    // Enforce steering velocity limits. We do this by taking the derivative of steering angle at
    // the current angle, and then backing out the maximum interpolant between start and goal
    // states. We remember the minimum across all modules, since that is the active constraint.
    for (int m = 0; m < NUM_MODULES; m++) {
      if (!vars.needToSteer) {
        vars.steeringOverride[m] = vars.prevModuleStates[m].angle;
        continue;
      }

      double maxThetaStep = vars.dt * maxSteerVelocityRadsPerSec;

      if (epsilonEquals(vars.prevModuleStates[m].speedMetersPerSecond, 0.0)) {
        // If module is stopped, we know that we will need to move straight to the final steering
        // angle, so limit based purely on rotation in place.
        if (epsilonEquals(vars.desiredModuleStates[m].speedMetersPerSecond, 0.0)) {
          // Goal angle doesn't matter. Just leave module at its current angle.
          vars.steeringOverride[m] = vars.prevModuleStates[m].angle;
          continue;
        }

        double requiredRots = angularDifference(vars.prev[m].radians(), vars.desired[m].radians());
        if (shouldFlipHeading(requiredRots)) {
          requiredRots =
              rotateBy(requiredRots, Rotation2d.k180deg.getCos(), Rotation2d.k180deg.getSin());
        }

        // radians bounds to +/- Pi.
        final double numStepsNeeded = Math.abs(requiredRots) / maxThetaStep;

        if (numStepsNeeded <= 1.0) {
          // Steer directly to goal angle.
          vars.steeringOverride[m] = vars.desiredModuleStates[m].angle;
        } else {
          // Adjust steering by max_theta_step.
          // there really is no way to avoid this allocation.
          Rotation2d adjusted =
              vars.prevModuleStates[m].angle.rotateBy(
                  Rotation2d.fromRadians(Math.signum(requiredRots) * maxThetaStep));
          vars.steeringOverride[m] = adjusted;
          vars.minS = 0.0;
        }
        continue;
      }

      // Enforce centripetal force limits to prevent sliding.
      // We do this by changing max_theta_step to the maximum change in heading over dt
      // that would create a large enough radius to keep the centripetal force under the
      // friction force.
      double maxHeadingChange =
          (vars.dt * wheelFrictionForce)
              / ((massKg / NUM_MODULES) * Math.abs(vars.prevModuleStates[m].speedMetersPerSecond));
      maxThetaStep = Math.min(maxThetaStep, maxHeadingChange);

      double s =
          findSteeringMaxS(
              vars.prev[m].vx,
              vars.prev[m].vy,
              vars.prev[m].radians(),
              vars.desired[m].vx,
              vars.desired[m].vy,
              vars.desired[m].radians(),
              maxThetaStep);
      vars.minS = Math.min(vars.minS, s);
    }
  }

  private void solveDriving(LocalVars vars) {
    // Enforce drive wheel torque limits
    MutTranslation2d chassisForceVec = new MutTranslation2d();
    MutTranslation2d moduleForceVec = new MutTranslation2d();
    double chassisTorque = 0.0;
    for (int m = 0; m < NUM_MODULES; m++) {
      // Use the current battery voltage since we won't be able to supply 12v if the
      // battery is sagging down to 11v, which will affect the max torque output
      double forwardCurrentDraw =
          driveMotor.getCurrent(
              // the motor should account for the gear ratio
              Math.abs(vars.prevModuleStates[m].speedMetersPerSecond / wheelRadiusMeters),
              vars.inputVoltage,
              driveSupplyCurrentLimitAmps,
              driveStatorCurrentLimitAmps);
      double reverseCurrentDraw =
          Math.abs(
              driveMotor.getCurrent(
                  // the motor should account for the gear ratio
                  Math.abs(vars.prevModuleStates[m].speedMetersPerSecond / wheelRadiusMeters),
                  -vars.inputVoltage,
                  driveSupplyCurrentLimitAmps,
                  driveStatorCurrentLimitAmps));
      double forwardModuleTorque = driveMotor.getTorque(forwardCurrentDraw);
      double reverseModuleTorque = driveMotor.getTorque(reverseCurrentDraw);

      double prevSpeed = vars.prevModuleStates[m].speedMetersPerSecond;
      vars.desiredModuleStates[m].optimize(vars.prevModuleStates[m].angle);
      double desiredSpeed = vars.desiredModuleStates[m].speedMetersPerSecond;

      int forceSign;
      Rotation2d forceAngle = vars.prevModuleStates[m].angle;
      double moduleTorque;
      if (epsilonEquals(prevSpeed, 0.0)
          || (prevSpeed > 0 && desiredSpeed >= prevSpeed)
          || (prevSpeed < 0 && desiredSpeed <= prevSpeed)) {
        moduleTorque = forwardModuleTorque;
        // Torque loss will be fighting motor
        moduleTorque -= torqueLoss;
        forceSign = 1; // Force will be applied in direction of module
        if (prevSpeed < 0) {
          forceAngle = forceAngle.plus(Rotation2d.k180deg);
        }
      } else {
        moduleTorque = reverseModuleTorque;
        // Torque loss will be helping the motor
        moduleTorque += torqueLoss;
        forceSign = -1; // Force will be applied in opposite direction of module
        if (prevSpeed > 0) {
          forceAngle = forceAngle.plus(Rotation2d.k180deg);
        }
      }

      // Limit torque to prevent wheel slip
      if (Math.abs(moduleTorque) > wheelFrictionForce * wheelRadiusMeters) {
        moduleTorque = wheelFrictionForce * wheelRadiusMeters;
      }

      double forceAtCarpet = moduleTorque / wheelRadiusMeters;
      // Translation2d moduleForceVec = new Translation2d(forceAtCarpet * forceSign, forceAngle);
      moduleForceVec.setPolar(forceAtCarpet * forceSign, forceAngle.getRadians());

      // Add the module force vector to the chassis force vector
      chassisForceVec.plusMut(moduleForceVec);

      // Calculate the torque this module will apply to the chassis
      if (!epsilonEquals(0, moduleForceVec.getNorm())) {
        Translation2d loc = moduleLocations[m];
        double unaryAngle = -angleOf(loc);
        double theta =
            rotateBy(angleOf(moduleForceVec), Math.cos(unaryAngle), Math.sin(unaryAngle));
        chassisTorque += forceAtCarpet * loc.getNorm() * Math.sin(theta);
      }
    }

    double chassisAngularAccel = chassisTorque / moiKgMetersSquared;

    double linearMult = 1.0;
    if (vars.constraintsOpt.isPresent()) {
      ChassisConstraints constraints = vars.constraintsOpt.get();
      double linearAccel = chassisForceVec.getNorm() / massKg;
      if (linearAccel > constraints.translation().maxAcceleration()) {
        linearMult = constraints.translation().maxAcceleration() / linearAccel;
      }
      chassisAngularAccel =
          MathUtil.clamp(
              chassisAngularAccel,
              -constraints.rotation().maxAcceleration(),
              constraints.rotation().maxAcceleration());
    }

    // Use kinematics to convert chassis accelerations to module accelerations
    ChassisSpeeds chassisAccel =
        new ChassisSpeeds(
            (chassisForceVec.getX() / massKg) * linearMult,
            (chassisForceVec.getY() / massKg) * linearMult,
            chassisAngularAccel);
    SwerveModuleState[] accelStates = kinematics.toSwerveModuleStates(chassisAccel);

    for (int m = 0; m < NUM_MODULES; m++) {
      if (vars.minS == 0.0) {
        // No need to carry on.
        break;
      }

      double maxVelStep = Math.abs(accelStates[m].speedMetersPerSecond * vars.dt);

      double vxMinS = (vars.desired[m].vx - vars.prev[m].vx) * vars.minS + vars.prev[m].vx;
      double vyMinS = (vars.desired[m].vy - vars.prev[m].vy) * vars.minS + vars.prev[m].vy;
      // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we
      // already know we can't go faster than that.
      double s = findDriveMaxS(vars.prev[m].vx, vars.prev[m].vy, vxMinS, vyMinS, maxVelStep);
      vars.minS = Math.min(vars.minS, s);
    }
  }
}
