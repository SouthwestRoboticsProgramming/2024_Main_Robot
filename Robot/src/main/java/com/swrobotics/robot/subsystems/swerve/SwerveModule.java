package com.swrobotics.robot.subsystems.swerve;

import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.config.CANAllocation;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// TODO: Implement for hardware modules
public interface SwerveModule {
    record Info(
            int driveId, int turnId, int encoderId,
            Translation2d position,
            NTEntry<Double> offset) {
        public Info(CANAllocation.SwerveIDs ids, double x, double y, NTEntry<Double> offset) {
            this(ids.drive, ids.turn, ids.encoder,
                    new Translation2d(x, y),
                    offset);
        }
    }

    void update(SwerveModuleState targetState);

    SwerveModuleState getCurrentState();
    SwerveModulePosition getCurrentPosition();

    Translation2d getRobotRelPosition();
}
