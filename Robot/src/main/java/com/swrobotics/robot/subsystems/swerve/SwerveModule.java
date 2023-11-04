package com.swrobotics.robot.subsystems.swerve;

import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// TODO: Implement for hardware modules
public interface SwerveModule {
    record Info(
            int driveId, int turnId, int encoderId,
            Translation2d position,
            NTEntry<Double> offset) {
        public Info(int driveId, int turnId, int encoderId, double x, double y, String name) {
            this(driveId, turnId, encoderId,
                    new Translation2d(x, y),
                    new NTDouble("Swerve/Modules/" + name + " Offset", 0).setPersistent());
        }
    }

    void update(SwerveModuleState targetState);

    SwerveModuleState getCurrentState();
    SwerveModulePosition getCurrentPosition();

    Translation2d getRobotRelPosition();
}
