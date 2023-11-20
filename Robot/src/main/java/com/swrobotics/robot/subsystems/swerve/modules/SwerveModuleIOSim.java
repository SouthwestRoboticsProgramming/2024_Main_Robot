package com.swrobotics.robot.subsystems.swerve.modules;

public class SwerveModuleIOSim implements SwerveModuleIO {

    @Override
    public void updateInputs(SwerveModuleIOInputs swerveModuleIOInputs) {
        // Execute demands
        // TODO

        // Read from harware and write to IOInputs
        swerveModuleIOInputs.drivePositionMeters += swerveModuleIOInputs.targetDriveVelocityMetersPerSec * 0.02;
        swerveModuleIOInputs.steerPositionRad = swerveModuleIOInputs.targetSteerPositionRad;
    }

    @Override
    public double getMaxVelocity() {
        return 5.0;
    }
    
}
