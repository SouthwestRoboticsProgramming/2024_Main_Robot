package com.swrobotics.robot.subsystems.swerve;

import com.swrobotics.messenger.client.MessengerClient;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.subsystems.swerve.pathfinding.ThetaStarPathfinder;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
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

public final class SwerveDrive extends SubsystemBase {
    public static record DriveRequest(
            ChassisSpeeds robotRelSpeeds,
            DriveRequestType driveRequestType,
            SteerRequestType steerRequestType
    ) {
        public DriveRequest(ChassisSpeeds robotRelSpeeds, DriveRequestType driveRequestType) {
            this(robotRelSpeeds, driveRequestType, SteerRequestType.MotionMagic);
        }
    }

    private static final double HALF_SPACING = Units.inchesToMeters(27 - 2.625 * 2); // Perimeter - MK4i inset // TODO: Set
    private static final SwerveModule.Info[] INFOS = {
            new SwerveModule.Info(CANAllocation.SWERVE_FL, HALF_SPACING, HALF_SPACING, NTData.FL_OFFSET, "Front Left"),
            new SwerveModule.Info(CANAllocation.SWERVE_FR, HALF_SPACING, -HALF_SPACING, NTData.FR_OFFSET, "Front Right"),
            new SwerveModule.Info(CANAllocation.SWERVE_BL, -HALF_SPACING, HALF_SPACING, NTData.BL_OFFSET, "Back Left"),
            new SwerveModule.Info(CANAllocation.SWERVE_BR, -HALF_SPACING, -HALF_SPACING, NTData.BR_OFFSET, "Back Right")
    };

    /** Meters per second */
    public static final double MAX_LINEAR_SPEED = 3.78; // TODO: Measure emperically

    
    private final AHRS gyro;
    private final SwerveModule[] modules;
    private final SwerveKinematics kinematics;
    private final SwerveEstimator estimator;

    private SwerveModulePosition[] prevPositions;
    private Rotation2d prevGyroAngle;

    private DriveRequest currentRequest;
    
    public SwerveDrive(FieldInfo fieldInfo, MessengerClient msg) {
        gyro = new AHRS(SPI.Port.kMXP);

        modules = new SwerveModule[INFOS.length];
        Translation2d[] positions = new Translation2d[INFOS.length];
        for (int i = 0; i < modules.length; i++) {
            SwerveModule.Info info = INFOS[i];

            SwerveModuleConstants moduleConstants = SWERVE_MODULE_BUILDER.createModuleConstants(info.turnId(), info.driveId(), info.encoderId(), info.offset().get(), info.position().getX(), info.position().getY(), false);
            if (RobotBase.isSimulation()) {
                moduleConstants = moduleConstants.withCANcoderOffset(0.25);
            }
            modules[i] = new SwerveModule(moduleConstants, info.name(), CANAllocation.CANIVORE_BUS);
            positions[i] = info.position();
        }

        this.kinematics = new SwerveKinematics(positions, MAX_LINEAR_SPEED);
        this.estimator = new SwerveEstimator(fieldInfo);

        prevPositions = null;
        currentRequest = null;

        // Configure pathing
        AutoBuilder.configureHolonomic(
                this::getEstimatedPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                (speeds) -> // TODO: Switch to velocity control once that is tuned
                        drive(new DriveRequest(speeds, DriveRequestType.OpenLoopVoltage)),
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

    public SwerveModulePosition[] getCurrentModulePositions(boolean refresh) {
        SwerveModulePosition[] positions = new SwerveModulePosition[INFOS.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = modules[i].getPosition(refresh);
        }
        return positions;
    }

    @AutoLogOutput(key = "Drive/Current Swerve Module Positions")
    public SwerveModulePosition[] getCurrentModulePositionsForLogging() {
        // No refresh here since it's already refreshed when we update the estimator
        return getCurrentModulePositions(false);
    }

    public void drive(DriveRequest request) {
        currentRequest = request;
    }

    @Override
    public void periodic() {
        // Apply the drive request
        if (currentRequest != null) {
            ChassisSpeeds robotRelSpeeds = ChassisSpeeds.discretize(currentRequest.robotRelSpeeds, 0.020);
            SwerveModuleState[] targetStates = kinematics.getStates(robotRelSpeeds);
            for (int i = 0; i < modules.length; i++) {
                modules[i].apply(targetStates[i], currentRequest.driveRequestType, currentRequest.steerRequestType);
            }

            currentRequest = null;
        }

        // Update estimator
        // Do refresh here, so we get the most up-to-date data
        SwerveModulePosition[] positions = getCurrentModulePositions(true);
        Rotation2d gyroAngle = gyro.getRotation2d();
        if (prevPositions != null) {
            Twist2d twist = kinematics.getTwistDelta(prevPositions, positions);
            Logger.recordOutput("Drive/Estimated Twist", twist);

            // We trust the gyro more than the kinematics estimate
            if (RobotBase.isReal() && gyro.isConnected()) {
                twist.dtheta = gyroAngle.getRadians() - prevGyroAngle.getRadians();
            }

            estimator.update(twist);
        }
        prevPositions = positions;
        prevGyroAngle = gyroAngle;
    }

    @Override
    public void simulationPeriodic() {
        for (SwerveModule module : modules) {
            module.updateSim(0.02, RobotController.getBatteryVoltage());
        }
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
}
