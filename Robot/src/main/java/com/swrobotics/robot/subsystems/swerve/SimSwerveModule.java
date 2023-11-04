package com.swrobotics.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class SimSwerveModule implements SwerveModule {
    private final Translation2d position;

    private SwerveModuleState target;
    private double distance;

    public SimSwerveModule(Info info) {
        this.position = info.position();
        distance = 0;
    }

    @Override
    public void update(SwerveModuleState targetState) {
        target = targetState;
        distance += target.speedMetersPerSecond * 0.02;
    }

    @Override
    public SwerveModuleState getCurrentState() {
        return target;
    }

    @Override
    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(distance, target.angle);
    }

    @Override
    public Translation2d getRobotRelPosition() {
        return position;
    }
}
