package com.swrobotics.robot.subsystems.swerve;

import com.swrobotics.messenger.client.MessengerClient;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.subsystems.swerve.pathfinding.ThetaStarPathfinder;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.robot.NTData;
import com.swrobotics.robot.config.CANAllocation;
import com.swrobotics.robot.subsystems.swerve.modules.SwerveModule;
import com.swrobotics.robot.subsystems.swerve.modules.SwerveModule3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.swrobotics.robot.subsystems.swerve.SwerveConstants.SWERVE_MODULE_BUILDER;

import java.util.Arrays;

public final class SwerveDrive extends SubsystemBase {
    private static final double HALF_SPACING = Units.inchesToMeters(20); // FIXME
    private static final SwerveModule.Info[] INFOS = {
            new SwerveModule.Info(CANAllocation.SWERVE_FL, HALF_SPACING, HALF_SPACING, NTData.FL_OFFSET, "Front Left"),
            new SwerveModule.Info(CANAllocation.SWERVE_FR, HALF_SPACING, -HALF_SPACING, NTData.FR_OFFSET, "Front Right"),
            new SwerveModule.Info(CANAllocation.SWERVE_BL, -HALF_SPACING, HALF_SPACING, NTData.BL_OFFSET, "Back Left"),
            new SwerveModule.Info(CANAllocation.SWERVE_BR, -HALF_SPACING, -HALF_SPACING, NTData.BR_OFFSET, "Back Right")
    };

    private static final double MAX_LINEAR_SPEED = 4.11; // FIXME: Different with Kraken

    
    private final AHRS gyro;
    private final SwerveModule3[] modules;
    private final SwerveKinematics kinematics;
    private final SwerveEstimator estimator;
    
    private SwerveModulePosition[] prevPositions;
    private SwerveModulePosition[] doublePrevPositions;
    private Rotation2d prevGyroAngle;
    
    public SwerveDrive(FieldInfo fieldInfo, MessengerClient msg) {
        gyro = new AHRS(SPI.Port.kMXP);

        modules = new SwerveModule3[INFOS.length];
        Translation2d[] positions = new Translation2d[INFOS.length];
        for (int i = 0; i < modules.length; i++) {
            SwerveModule.Info info = INFOS[i];

            SwerveModuleConstants moduleConstants = SWERVE_MODULE_BUILDER.createModuleConstants(info.turnId(), info.driveId(), info.encoderId(), info.offset().get(), info.position().getX(), info.position().getY(), false);
            if (RobotBase.isSimulation()) {
                moduleConstants = moduleConstants.withCANcoderOffset(0.25);
            }
            modules[i] = new SwerveModule3(moduleConstants, CANAllocation.CANIVORE_BUS);
            positions[i] = info.position();
        }

        this.kinematics = new SwerveKinematics(positions, MAX_LINEAR_SPEED);
        this.estimator = new SwerveEstimator(fieldInfo);

        prevPositions = null;


        // Configure pathing
        AutoBuilder.configureHolonomic(
            this::getEstimatedPose,
            this::setPose,
            this::getRobotRelativeSpeeds,
            this::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(8.0), new PIDConstants(4.0, 0.0), MAX_LINEAR_SPEED, Math.hypot(HALF_SPACING, HALF_SPACING), new ReplanningConfig(), 0.020),
            this);

//        Pathfinding.setPathfinder(new LocalADStar());
        Pathfinding.setPathfinder(new ThetaStarPathfinder(msg));
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
              Logger.recordOutput(
                  "Drive/Trajectory", activePath.toArray(new Pose2d[0]));
                FieldView.pathPlannerPath.setPoses(activePath);
        });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
                Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                FieldView.pathPlannerSetpoint.setPose(targetPose);
        });
    }

    @AutoLogOutput(key = "Drive/Current Swerve Module States")
    public SwerveModuleState[] getCurrentModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[INFOS.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = modules[i].getCurrentState();
        }
        return states;
    }

    @AutoLogOutput(key = "Drive/Target Swerve Module States")
    public SwerveModuleState[] getTargetModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[INFOS.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = modules[i].getTargetState();
        }
        return states;
    }

    @AutoLogOutput(key = "Drive/Current Swerve Module Positions")
    public SwerveModulePosition[] getCurrentModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[INFOS.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = modules[i].getPosition(true);
        }
        return positions;
    }

    // TODO: Better way of selecting between manual/auto input
    // TODO: Split some of this into periodic(), since drive is not guaranteed to be called every time
    public void drive(ChassisSpeeds robotRelSpeeds) {
        robotRelSpeeds = ChassisSpeeds.discretize(robotRelSpeeds, 0.020);
        SwerveModuleState[] targetStates = kinematics.getStates(robotRelSpeeds);
        for (int i = 0; i < modules.length; i++) {
            modules[i].apply(targetStates[i], DriveRequestType.OpenLoopVoltage);
        }
        SwerveModulePosition[] positions = getCurrentModulePositions();

        Rotation2d gyroAngle = gyro.getRotation2d();
        if (prevPositions != null) {
            // if (doublePrevPositions != null) {

                // System.out.println(getCurrentModulePositions()[0] == prevPositions[0]);
    
                Twist2d twist = kinematics.getTwistDelta(prevPositions, positions);
                // System.out.println(positions[0] == prevPositions[0]);
    
                // We trust the gyro more than the kinematics estimate
                if (RobotBase.isReal() && gyro.isConnected()) {
                    twist.dtheta = gyroAngle.getRadians() - prevGyroAngle.getRadians();
                }
    
                estimator.update(twist);
            // }
            // doublePrevPositions = prevPositions;
        }
        prevPositions = positions;
        prevGyroAngle = gyroAngle;
    }

    @AutoLogOutput(key = "Pose Estimate")
    public Pose2d getEstimatedPose() {
        return estimator.getEstimatedPose();
    }

    public void setPose(Pose2d newPose) {
        estimator.resetPose(newPose);
    }

    public void setRotation(Rotation2d newRotation) {
        estimator.resetPose(new Pose2d(getEstimatedPose().getTranslation(), newRotation));
    }

    @AutoLogOutput(key = "Drive/Robot Rel Velocity")
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getCurrentModuleStates());
    }


    @Override
    public void simulationPeriodic() {
        for (SwerveModule3 module : modules) {
            module.updateSim(0.02, RobotController.getBatteryVoltage());
        }
    }
}
