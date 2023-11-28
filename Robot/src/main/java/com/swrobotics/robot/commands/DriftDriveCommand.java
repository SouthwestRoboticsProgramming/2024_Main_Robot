package com.swrobotics.robot.commands;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.pathplanner.lib.util.ChassisSpeedsRateLimiter;
import com.swrobotics.mathlib.Angle;
import com.swrobotics.mathlib.CWAngle;
import com.swrobotics.mathlib.Vec2d;
import com.swrobotics.robot.control.ControlBoard;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriftDriveCommand extends Command {
    private final SwerveDrive drive;
    private final ControlBoard input;
    private final ChassisSpeedsRateLimiter limiter = new ChassisSpeedsRateLimiter(1, Double.MAX_VALUE, new ChassisSpeeds());
    private final ChassisSpeedsRateLimiter driftLimiter = new ChassisSpeedsRateLimiter(1, Double.MAX_VALUE, new ChassisSpeeds());

    private final LoggedDashboardNumber driftCoefficient = new LoggedDashboardNumber("Drive/Drift Coefficient", 1.5);
    private final LoggedDashboardNumber gripVelocityLimit = new LoggedDashboardNumber("Drive/Grip Velocity Limit", 2.0);
    private final LoggedDashboardBoolean isDrifting = new LoggedDashboardBoolean("Drive/Drifting", false);

    public DriftDriveCommand(SwerveDrive drive, ControlBoard controlBoard) {
        this.drive = drive;
        input = controlBoard;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        Vec2d rawTranslation = input.getDriveTranslation();
        double rawRotation = input.getDriveRotation();

        // Scale inputs
        Vec2d translation = rawTranslation.mul(4.11);
        Angle rotation =
                CWAngle.rad(Math.PI).mul(rawRotation);

        /*
        limiter.setRateLimits(normalLimit.get(), Double.MAX_VALUE);
        driftLimiter.setRateLimits(driftLimit.get(), Double.MAX_VALUE);

        /*
         * Limit the acceleration
         * Limiting the direction that isn't directly controlled gives the drift effect
         */ /*
        isDrifting.set(true);
        ChassisSpeeds demand = new ChassisSpeeds(translation.x, 0.0, rotation.ccw().rad());
        demand = ChassisSpeeds.fromRobotRelativeSpeeds(demand, drive.getEstimatedPose().getRotation());
        demand = limiter.calculate(demand);
        ChassisSpeeds driftDemand = driftLimiter.calculate(demand);
        driftDemand = ChassisSpeeds.fromFieldRelativeSpeeds(driftDemand, drive.getEstimatedPose().getRotation());
        demand = ChassisSpeeds.fromFieldRelativeSpeeds(demand, drive.getEstimatedPose().getRotation());
        demand.vyMetersPerSecond = driftDemand.vyMetersPerSecond;
        
        if (demand.vyMetersPerSecond < gripVelocityLimit.get()) {
            isDrifting.set(false);
            demand.vyMetersPerSecond = 0;
        }
        */
        ChassisSpeeds current = drive.getCurrentRobotRelativeVelocity();
        double centripedalAcceleration = current.vxMetersPerSecond * current.omegaRadiansPerSecond; // Ac = v * theta
        double vyMetersPerSecond = current.vyMetersPerSecond + centripedalAcceleration;
        vyMetersPerSecond *= driftCoefficient.get();

        // The back isn't necesesarily breaking traction
        isDrifting.set(true);
        if (current.vxMetersPerSecond < gripVelocityLimit.get()) {
            isDrifting.set(false);
            vyMetersPerSecond = 0.0;
        }

        ChassisSpeeds demand = new ChassisSpeeds(translation.x, -vyMetersPerSecond, rotation.ccw().rad());

        drive.drive(
            demand
        );

    }
}
