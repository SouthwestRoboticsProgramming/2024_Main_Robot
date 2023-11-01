package com.swrobotics.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
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

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        // Limit the speeds using maximum accelerations
        
        // kinematicLimits.limitSpeeds(new ChassisSpeeds(1, 1, 1), new ChassisSpeeds());
        // this.speeds = speeds;
        Pose2d robotPoseVel =
                new Pose2d(
                        speeds.vxMetersPerSecond * 0.02,
                        speeds.vyMetersPerSecond * 0.02,
                        Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * 0.02));
        Twist2d twistVel = new Pose2d().log(robotPoseVel);
        speeds =
                new ChassisSpeeds(
                        twistVel.dx / 0.02,
                        twistVel.dy / 0.02,
                        twistVel.dtheta / 0.02);
        this.speeds = kinematicLimits.limitSpeeds(speeds, getChassisSpeeds());

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
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Units.feetToMeters(18.0));

        setSwerveModuleStates(states);

        // Simulate gyroscope behavior (badly)q
        ChassisSpeeds estimatedSpeeds = kinematics.toChassisSpeeds(getSwerveModuleStates());
        gyroAngle = gyroAngle.plus(new Rotation2d(estimatedSpeeds.omegaRadiansPerSecond * 0.02));

        odometry.update(gyroAngle, getSwerveModulePositions());
        field.setRobotPose(odometry.getPoseMeters());
        // System.out.println(kinematicLimits.getNetAcceleration().getNorm());
    }
}
