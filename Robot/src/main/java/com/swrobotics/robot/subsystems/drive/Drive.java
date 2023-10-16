package com.swrobotics.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final SwerveModule[] modules =
            new SwerveModule[] {
                new SwerveModule(), new SwerveModule(), new SwerveModule(), new SwerveModule()
            };

    private Rotation2d gyroAngle = new Rotation2d();
    private KinematicLimits kinematicLimits = new KinematicLimits();
    private ChassisSpeeds speeds = new ChassisSpeeds();

    private final Field2d field = new Field2d();

    public Drive() {
        System.out.println("[Init] Creating Drive");

        kinematics = DriveConstants.kinematics;
        odometry = new SwerveDriveOdometry(kinematics, gyroAngle, getSwerveModulePositions());

        SmartDashboard.putData("Field", field);
    }

    public void resetGyro() {}

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public void setSwerveModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < states.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public KinematicLimits getKinematicLimits() {
        return kinematicLimits;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    @Override
    public void periodic() {
        // Send states to modules
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        // TODO: Change to better desaturateWheelSpeeds
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 4.11);

        setSwerveModuleStates(states);

        // Simulate gyroscope behavior (badly)
        ChassisSpeeds estimatedSpeeds = kinematics.toChassisSpeeds(getSwerveModuleStates());
        gyroAngle.plus(new Rotation2d(estimatedSpeeds.omegaRadiansPerSecond * 0.02));

        odometry.update(gyroAngle, getSwerveModulePositions());
        field.setRobotPose(odometry.getPoseMeters());
    }

    public static class KinematicLimits {
        public double maxDriveVelocity = 4.11; // m/s
        public double maxAccel = Double.MAX_VALUE; // m/s^2
        public double maxAnglularVelocity = 11.5; // rad/s
        public double maxAngularAccel = Double.MAX_VALUE; // rad/s^2
    }
}
