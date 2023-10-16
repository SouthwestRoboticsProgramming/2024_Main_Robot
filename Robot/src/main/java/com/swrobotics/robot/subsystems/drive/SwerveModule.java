package com.swrobotics.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private SwerveModuleState targetState = new SwerveModuleState();

    private double simulatedDistance = 0.0;

    public void setState(SwerveModuleState state) {
        targetState = state;

        simulatedDistance += targetState.speedMetersPerSecond * 0.02;
    }

    public SwerveModuleState getState() {
        return targetState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    public Rotation2d getAngle() {
        return targetState.angle;
    }

    public double getDistance() {
        return simulatedDistance;
    }
}
