package com.swrobotics.robot.subsystems.swerve;

import com.swrobotics.messenger.client.MessengerClient;
import com.swrobotics.robot.logging.FieldView;
import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.robot.NTData;
import com.swrobotics.robot.config.CANAllocation;
import com.swrobotics.robot.subsystems.swerve.modules.SwerveModule;
import com.swrobotics.robot.subsystems.swerve.modules.SwerveModuleIORealisticSim;
import com.swrobotics.robot.subsystems.swerve.modules.SwerveModuleIOTalonFX;

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
            new SwerveModule.Info(CANAllocation.SWERVE_FL, HALF_SPACING, HALF_SPACING, NTData.FL_OFFSET, "Front Left"),
            new SwerveModule.Info(CANAllocation.SWERVE_FR, HALF_SPACING, -HALF_SPACING, NTData.FR_OFFSET, "Front Right"),
            new SwerveModule.Info(CANAllocation.SWERVE_BL, -HALF_SPACING, HALF_SPACING, NTData.BL_OFFSET, "Back Left"),
            new SwerveModule.Info(CANAllocation.SWERVE_BR, -HALF_SPACING, -HALF_SPACING, NTData.BR_OFFSET, "Back Right")
    };

    private final AHRS gyro;
    private final SwerveModule[] modules;
    private final SwerveKinematics kinematics;
    private final SwerveEstimator estimator;

    private SwerveModulePosition[] prevPositions;
    private Rotation2d prevGyroAngle;

    public SwerveDrive(FieldInfo fieldInfo, MessengerClient msg) {
        gyro = new AHRS(SPI.Port.kMXP);

        modules = new SwerveModule[INFOS.length];
        Translation2d[] positions = new Translation2d[INFOS.length];
        for (int i = 0; i < modules.length; i++) {
            SwerveModule.Info info = INFOS[i];
            if (RobotBase.isSimulation()) {
                modules[i] = new SwerveModule(new SwerveModuleIORealisticSim(), INFOS[i]);
            } else {
                modules[i] = new SwerveModule(new SwerveModuleIOTalonFX(INFOS[i], CANAllocation.CANIVORE_BUS), INFOS[i]);
            }
            positions[i] = info.position();
        }

        double minMax = Double.POSITIVE_INFINITY;
        for (SwerveModule module : modules) {
            minMax = Math.min(module.getMaxVelocity(), minMax);
        }

        this.kinematics = new SwerveKinematics(positions, minMax);
        this.estimator = new SwerveEstimator(fieldInfo);

        prevPositions = null;

        // Configure pathing
        AutoBuilder.configureHolonomic(
            this::getEstimatedPose,
            this::setPose,
            this::getRobotRelativeSpeeds,
            this::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(8.0), new PIDConstants(4.0, 0.0), minMax, Math.hypot(HALF_SPACING, HALF_SPACING), new ReplanningConfig(), 0.020),
            this);

        Pathfinding.setPathfinder(new LocalADStar()); // TODO: Theta*
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

    public SwerveModuleState[] getCurrentModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[INFOS.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = modules[i].getCurrentState();
        }
        return states;
    }

    public SwerveModuleState[] getTargetModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[INFOS.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = modules[i].getTargetState();
        }
        return states;
    }

    public SwerveModulePosition[] getCurrentModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[INFOS.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = modules[i].getCurrentPosition();
        }
        return positions;
    }

    // TODO: Better way of selecting between manual/auto input
    // TODO: Split some of this into periodic(), since drive is not guaranteed to be called every time
    public void drive(ChassisSpeeds robotRelSpeeds) {
        System.out.println("Driving: " + robotRelSpeeds);
        robotRelSpeeds = ChassisSpeeds.discretize(robotRelSpeeds, 0.020);
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

    public void setPose(Pose2d newPose) {
        estimator.resetPose(newPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getCurrentModuleStates());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Pose estimate", getEstimatedPose());
        for (SwerveModule module: modules) {
            module.update();
        }

        Logger.recordOutput("Drive/Current Swerve Module States", getCurrentModuleStates());
        Logger.recordOutput("Drive/Target Swerve Module States", getTargetModuleStates());

    }

    // private SwerveModuleState optimizeSwerveModuleState(SwerveModuleState targetState, SwerveModuleState currentState) {
    //     Rotation2d currentAngle = currentState.angle;
    //     Rotation2d targetAngle = targetState.angle;
    //     Rotation2d inverseAngle = targetAngle.plus(Rotation2d.fromDegrees(180));

    // }
}
