package com.swrobotics.robot.commands;

import com.swrobotics.lib.net.NTUtil;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public final class AimAtPointCommand extends Command {
    private final SwerveDrive drive;
    private final Supplier<Translation2d> targetSupplier;

    private final PIDController pid;

    public AimAtPointCommand(SwerveDrive drive, Supplier<Translation2d> targetSupplier) {
        this.drive = drive;
        this.targetSupplier = targetSupplier;

        pid = NTUtil.tunablePID(NTData.DRIVE_AIM_KP, NTData.DRIVE_AIM_KI, NTData.DRIVE_AIM_KD);
        pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getEstimatedPose();
        Translation2d robotPos = robotPose.getTranslation();

        Translation2d target = targetSupplier.get();

        // Guaranteed to be in [-PI, PI]
        double angleToTarget = target.minus(robotPos).getAngle().getRadians();
        double currentAngle = MathUtil.wrap(robotPose.getRotation().getRadians(), -Math.PI, Math.PI);

        double max = NTData.DRIVE_AIM_MAX_TURN.get() * MathUtil.TAU;
        double output = pid.calculate(currentAngle, angleToTarget);
        output = MathUtil.clamp(output, -max, max);

        drive.turn(new SwerveDrive.TurnRequest(SwerveDrive.AIM_PRIORITY, new Rotation2d(output)));
    }

    public boolean isInTolerance(double tolRotations) {
        Pose2d robotPose = drive.getEstimatedPose();
        Translation2d robotPos = robotPose.getTranslation();
        Translation2d target = targetSupplier.get();

        double angleToTarget = target.minus(robotPos).getAngle().getRadians();
        double currentAngle = MathUtil.wrap(robotPose.getRotation().getRadians(), -Math.PI, Math.PI);

        return MathUtil.absDiffRad(currentAngle, angleToTarget) < tolRotations * MathUtil.TAU;
    }
}
