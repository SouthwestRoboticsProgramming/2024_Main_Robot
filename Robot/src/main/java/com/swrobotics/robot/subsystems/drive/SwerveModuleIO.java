package com.swrobotics.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelcius = 0.0;

        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTempCelcius = 0.0;
    }

    /** Update the set of loggable inputs */
    public default void updateInputa(SwerveModuleIOInputs inputs) {}

    // TODO: Motor control

    /** Enable or disable brake mode on the drive motor */
    public default void setDriveBrakeMode(boolean enable) {}

    /** Enable or disable brake mode on the turn motor */
    public default void setTurnBrakeMode(boolean enable) {}
}
