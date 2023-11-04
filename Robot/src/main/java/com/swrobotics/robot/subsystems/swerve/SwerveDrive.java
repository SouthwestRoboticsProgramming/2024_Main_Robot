package com.swrobotics.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class SwerveDrive extends SubsystemBase {
    private static final double HALF_SPACING = Units.inchesToMeters(20); // FIXME
    private static final SimSwerveModule.Info[] INFOS = {
            new SwerveModule.Info(9, 5, 1, HALF_SPACING, HALF_SPACING, "Front Left"),
            new SwerveModule.Info(10, 6, 2, HALF_SPACING, -HALF_SPACING, "Front Right"),
            new SwerveModule.Info(11, 7, 3, -HALF_SPACING, HALF_SPACING, "Back Left"),
            new SwerveModule.Info(12, 8, 4, -HALF_SPACING, -HALF_SPACING, "Back Right")
    };

    private final AHRS gyro;
    private final SimSwerveModule[] modules;
    private final SwerveKinematics kinematics;
    private final SwerveEstimator estimator;

    private SwerveModulePosition[] prevPositions;
    private Rotation2d prevGyroAngle;

    public SwerveDrive() {
        gyro = new AHRS(SPI.Port.kMXP);

        modules = new SimSwerveModule[INFOS.length];
        Translation2d[] positions = new Translation2d[INFOS.length];
        for (int i = 0; i < modules.length; i++) {
            SimSwerveModule.Info info = INFOS[i];
            modules[i] = new SimSwerveModule(info);
            positions[i] = info.position();
        }

        this.kinematics = new SwerveKinematics(positions);
        this.estimator = new SwerveEstimator();

        prevPositions = null;
    }

    public void drive(ChassisSpeeds robotRelSpeeds) {
        SwerveModuleState[] targetStates = kinematics.getStates(robotRelSpeeds);
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            modules[i].update(targetStates[i]);
            positions[i] = modules[i].getCurrentPosition();
        }

        Rotation2d gyroAngle = gyro.getRotation2d();
        if (prevPositions != null) {
            Twist2d twist = kinematics.getTwistDelta(prevPositions, positions);

            // We trust the gyro more than the kinematics estimate
            if (RobotBase.isReal())
                twist.dtheta = gyroAngle.getRadians() - prevGyroAngle.getRadians();

            estimator.addDriveMovement(twist);
        }
        prevPositions = positions;
        prevGyroAngle = gyroAngle;
    }

    public Pose2d getEstimatedPose() {
        return estimator.getEstimatedPose();
    }
}
