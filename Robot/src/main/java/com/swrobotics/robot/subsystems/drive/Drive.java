package com.swrobotics.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final SwerveModule[] modules = new SwerveModule[] {
        new SwerveModule(), new SwerveModule(), new SwerveModule(), new SwerveModule()
    };

    private Rotation2d gyroAngle = new Rotation2d();

    private final Field2d field = new Field2d();

    public Drive() {
        System.out.println("[Init] Creating Drive");

        kinematics = DriveConstants.kinematics;
        odometry = new SwerveDriveOdometry(kinematics, gyroAngle, getSwerveModulePositions());

        SmartDashboard.putData("Field", field);
    }

    public void resetGyro() {
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {

    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    @Override
    public void periodic() {
        odometry.update(gyroAngle, getSwerveModulePositions());
        field.setRobotPose(odometry.getPoseMeters());
    }
}
