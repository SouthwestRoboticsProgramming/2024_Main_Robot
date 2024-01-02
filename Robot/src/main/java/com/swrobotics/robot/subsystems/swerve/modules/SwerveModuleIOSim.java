package com.swrobotics.robot.subsystems.swerve.modules;

public class SwerveModuleIOSim implements SwerveModuleIO {

    @Override
    public void updateInputs(SwerveModuleIOInputs swerveModuleIOInputs) {
        // Read from harware and write to IOInputs
        swerveModuleIOInputs.drivePositionMeters += swerveModuleIOInputs.targetDriveVelocityMetersPerSec * 0.02;
        swerveModuleIOInputs.steerPositionRad = swerveModuleIOInputs.targetSteerPositionRad;
    }
}
