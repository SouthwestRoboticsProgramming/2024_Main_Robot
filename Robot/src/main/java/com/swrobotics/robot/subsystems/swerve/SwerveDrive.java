package com.swrobotics.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.swrobotics.lib.field.FieldInfo;
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
    private static final SwerveModule.Info[] INFOS = {
            new SwerveModule.Info(9, 5, 1, HALF_SPACING, HALF_SPACING, "Front Left"),
            new SwerveModule.Info(10, 6, 2, HALF_SPACING, -HALF_SPACING, "Front Right"),
            new SwerveModule.Info(11, 7, 3, -HALF_SPACING, HALF_SPACING, "Back Left"),
            new SwerveModule.Info(12, 8, 4, -HALF_SPACING, -HALF_SPACING, "Back Right")
    };

    private final AHRS gyro;
    private final SwerveModule[] modules;
    private final SwerveKinematics kinematics;
    private final SwerveEstimator estimator;

    private SwerveModulePosition[] prevPositions;
    private Rotation2d prevGyroAngle;

    public SwerveDrive(FieldInfo fieldInfo) {
        gyro = new AHRS(SPI.Port.kMXP);

        modules = new SwerveModule[INFOS.length];
        Translation2d[] positions = new Translation2d[INFOS.length];
        for (int i = 0; i < modules.length; i++) {
            SwerveModule.Info info = INFOS[i];
            // TODO: If sim
            modules[i] = new SwerveModule(new SwerveModuleIOSim(), INFOS[i]);
            positions[i] = info.position();
        }

        this.kinematics = new SwerveKinematics(positions);
        this.estimator = new SwerveEstimator(fieldInfo);

        prevPositions = null;
    }

    public void drive(ChassisSpeeds robotRelSpeeds) {
        SwerveModuleState[] targetStates = kinematics.getStates(robotRelSpeeds);
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(targetStates[i]);
            positions[i] = modules[i].getCurrentPosition();
        }

        Rotation2d gyroAngle = gyro.getRotation2d();
        if (prevPositions != null) {
            Twist2d twist = kinematics.getTwistDelta(prevPositions, positions);

            // We trust the gyro more than the kinematics estimate
            if (RobotBase.isReal())
                twist.dtheta = gyroAngle.getRadians() - prevGyroAngle.getRadians();

            estimator.update(twist);
        }
        prevPositions = positions;
        prevGyroAngle = gyroAngle;
    }

    public Pose2d getEstimatedPose() {
        return estimator.getEstimatedPose();
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("Pose estimate", getEstimatedPose());
        for (SwerveModule module: modules) {
            module.update();
        }
    }

    // private SwerveModuleState optimizeSwerveModuleState(SwerveModuleState targetState, SwerveModuleState currentState) {
    //     Rotation2d currentAngle = currentState.angle;
    //     Rotation2d targetAngle = targetState.angle;
    //     Rotation2d inverseAngle = targetAngle.plus(Rotation2d.fromDegrees(180));

    // }
}
