package com.swrobotics.robot.subsystems.swerve.modules;

import com.swrobotics.robot.logging.AutoLoggedInputs;

import com.ctre.phoenix6.BaseStatusSignal;

/**
 * Connects the software to the hardware and directly receives data and/or sends control data to the swerve drive module.
 */
public interface SwerveModuleIO {
    /**
     * Updates the swerve module values from the data received by the module.
     *
     * @param swerveModuleIOInputs Has the current data of the swerve drive module.
     */
    public void updateInputs(SwerveModuleIOInputs swerveModuleIOInputs);

    default void resetToAbsoluteAngle() {}

    public double getMaxVelocity();

    default BaseStatusSignal[] getCTRESignals() {
        return new BaseStatusSignal[0];
    }

    /**
     * Holds data that can be read from the corresponding swerve drive module IO implementation.
     */
//    @AutoLog
    class SwerveModuleIOInputs extends AutoLoggedInputs {
        // Drive Inputs
        public double drivePositionRad = 0;
        /**
         * Drive motor positioning
         */
        public double drivePositionMeters = 0;
        /**
         * Drive motor speed
         */
        public double driveVelocityMetersPerSec = 0;
        /**
         * Amps going to drive motor
         */
        public double driveCurrentDrawAmps = 0;
        /**
         * Volts being sent to the drive motor
         */
        public double driveAppliedVolts = 0;

        // Steering Inputs
        /**
         * Steering position angle
         */
        public double steerPositionTicks = 0;

        public double steerPositionRad = 0;

        public double steerPositionDeg = 0;
        /**
         * Steering motor speed
         */
        public double steerVelocityRadPerSec = 0;
        /**
         * Amps going to steer motor.
         */
        public double steerCurrentDrawAmps = 0;
        /**
         * Volts being sent to the steer motor
         */
        public double steerAppliedVolts = 0;

        // Steering Encoder Inputs
        /**
         * Absolute position. Angle stays the same at all times (as opposed to the motor encoders which update the angles based off of the starting angle).
         */
        public double steerAbsolutePosition = 0;

        // Drive Outputs
        public double targetDriveVelocityMetersPerSec = 0;

        // Steer Outputs
        public double targetSteerPositionRad = 0;
    }
}