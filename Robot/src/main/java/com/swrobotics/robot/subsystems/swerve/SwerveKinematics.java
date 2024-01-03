package com.swrobotics.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class SwerveKinematics {
    private final SwerveDriveKinematics kinematics;
    private final double maxVelocity;

    public SwerveKinematics(Translation2d[] modulePositions, double maxVelocity) {
        kinematics = new SwerveDriveKinematics(modulePositions);
        this.maxVelocity = maxVelocity;
    }

    public SwerveModuleState[] getStates(ChassisSpeeds robotRelSpeeds) {
        final double periodicTime = 0.02;

        // "Borrowed" from team 254
        Pose2d robotPoseVel =
                new Pose2d(
                        robotRelSpeeds.vxMetersPerSecond * periodicTime,
                        robotRelSpeeds.vyMetersPerSecond * periodicTime,
                        Rotation2d.fromRadians(robotRelSpeeds.omegaRadiansPerSecond * periodicTime));
        Twist2d twistVel = new Pose2d().log(robotPoseVel);
        robotRelSpeeds =
                new ChassisSpeeds(
                        twistVel.dx / periodicTime,
                        twistVel.dy / periodicTime,
                        twistVel.dtheta / periodicTime);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(robotRelSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);

        return states;
    }

    public Twist2d getTwistDelta(SwerveModulePosition[] startPositions, SwerveModulePosition[] endPositions) {
        SwerveModulePosition[] deltas = new SwerveModulePosition[startPositions.length];
        for (int i = 0; i < deltas.length; i++) {
            deltas[i] = new SwerveModulePosition(
                    endPositions[i].distanceMeters - startPositions[i].distanceMeters,
                    endPositions[i].angle);
        }

        return kinematics.toTwist2d(deltas);
    }

    public ChassisSpeeds toChassisSpeeds(SwerveModuleState[] moduleStates) {
        return kinematics.toChassisSpeeds(moduleStates);
    }

    public SwerveDriveKinematics toSwerveDriveKinematics() {
        return kinematics;
    }
}
