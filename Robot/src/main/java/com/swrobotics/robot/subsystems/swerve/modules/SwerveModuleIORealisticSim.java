package com.swrobotics.robot.subsystems.swerve.modules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Implementation of the Swerve Module IO interface. Simulating the drive train of a robot through software instead of hardware.
 */
public class SwerveModuleIORealisticSim implements SwerveModuleIO {
    // Settings to replicate a real life Falcon500 motor
    /**
     * Wheel radius of an actual Falcon500 Swerve Module
     */
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
    /**
     * Simulated drive motor using the accurate gearing ratios and moment of inertia
     */
    private final FlywheelSim driveSim = new FlywheelSim(DCMotor.getKrakenX60Foc(1), 8.14 /* L1 */, 0.025);
    /**
     * Simulated steer motor using the accurate gearing ratios and moment of inertia
     */
    private final FlywheelSim steerSim = new FlywheelSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.004096955);
    /**
     * PID Controller for drive motor. Will do our target drive velocity to voltage calculation
     */
    private final PIDController driveFeedback = new PIDController(5, 0, 0);
    /**
     * PID Controller for steer motor. Will do our target angular position to voltage calculation
     */
    private final PIDController steerFeedback = new PIDController(10, 0, 0);
    /**
     * Represents the swerve module encoder's absolute value
     */
    private double steerRelativePositionRad = 0;
    /**
     * Represents a starting angular position value for our MOTOR encoder
     */
    private double steerAbsolutePositionRad = Math.random() * 2.0 * Math.PI;

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Set outputs
        driveFeedback.setSetpoint(inputs.targetDriveVelocityMetersPerSec);
        steerFeedback.setSetpoint(inputs.targetSteerPositionRad);

        // Calculating target data to voltage data
        double driveAppliedVolts = driveFeedback.calculate(inputs.driveVelocityMetersPerSec);
        driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
        double steerAppliedVolts = steerFeedback.calculate(inputs.steerPositionRad);
        steerAppliedVolts = MathUtil.clamp(steerAppliedVolts, -12.0, 12.0);

        // Applying calculated voltage data to simulated motors
        driveSim.setInputVoltage(driveAppliedVolts);
        steerSim.setInputVoltage(steerAppliedVolts);

        // Updates every set amount of seconds (replicating a robot, we are using the default time between updates for
        // the robot)
        driveSim.update(0.020);
        steerSim.update(0.020);

        // Adding the difference in angles between the absolute and relative angles to the current angle (correcting for
        // the offset)
        double angleDiffRad = steerSim.getAngularVelocityRadPerSec() * 0.020;
        steerRelativePositionRad += angleDiffRad;
        steerAbsolutePositionRad += angleDiffRad;

        // Limiting the domain of the absolute angular position to be in between 0 and 2π
        // steerAbsolutePositionRad = MathUtil.inputModulus(steerAbsolutePositionRad, 0, 2 * Math.PI);

        // Setting the inputs within the IO class
        inputs.drivePositionRad =
                inputs.drivePositionRad + (driveSim.getAngularVelocityRadPerSec() * 0.020);
        inputs.drivePositionMeters +=
                (driveSim.getAngularVelocityRadPerSec() * 0.020) * WHEEL_RADIUS_METERS;
        inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * WHEEL_RADIUS_METERS;
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentDrawAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.steerPositionRad = steerRelativePositionRad;
        inputs.steerPositionDeg = Math.toDegrees(inputs.steerPositionRad);
        inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVolts = steerAppliedVolts;
        inputs.steerCurrentDrawAmps = Math.abs(steerSim.getCurrentDrawAmps());

        inputs.steerAbsolutePosition = steerAbsolutePositionRad;
    }
}
