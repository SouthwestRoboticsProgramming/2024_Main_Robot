package com.swrobotics.robot.commands;

import com.swrobotics.lib.net.NTUtil;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.subsystems.speaker.ShooterSubsystem;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public final class AimTowardsLobCommand extends Command {
    private final SwerveDrive drive;
    private final ShooterSubsystem shooter;

    private final PIDController pid;
    private double errorRad;

    public AimTowardsLobCommand(SwerveDrive drive, ShooterSubsystem shooter) {
        this.drive = drive;
        this.shooter = shooter;

        pid = NTUtil.tunablePID(NTData.DRIVE_AIM_KP, NTData.DRIVE_AIM_KD);
        pid.enableContinuousInput(-Math.PI, Math.PI);

        errorRad = Double.POSITIVE_INFINITY;
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getEstimatedPose();
        Translation2d robotPos = robotPose.getTranslation();

        Translation2d target = shooter.getLobZonePosition();
        Rotation2d angleToTarget = target.minus(robotPos).getAngle();

        double correctionRad = 0;

        double setpointAngle = MathUtil.wrap(angleToTarget.getRadians() + correctionRad, -Math.PI, Math.PI);
        double currentAngle = MathUtil.wrap(robotPose.getRotation().getRadians(), -Math.PI, Math.PI);

        double max = NTData.DRIVE_AIM_MAX_TURN.get() * MathUtil.TAU;
        double output = pid.calculate(currentAngle, setpointAngle);
        errorRad = pid.getPositionError();
        output = MathUtil.clamp(output, -max, max);

        drive.turn(new SwerveDrive.TurnRequest(SwerveDrive.AIM_PRIORITY, new Rotation2d(output)));
    }

    public boolean isInTolerance(double tolRotations) {
        return errorRad < tolRotations * MathUtil.TAU;
    }
}
