package com.swrobotics.robot.commands;

import com.swrobotics.lib.net.NTUtil;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.subsystems.speaker.ShooterSubsystem;
import com.swrobotics.robot.subsystems.speaker.aim.AimCalculator;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public final class AimTowardsSpeakerCommand extends Command {
    private final SwerveDrive drive;
    private final ShooterSubsystem shooter;

    private final PIDController pid;
    private double errorRad;

    public AimTowardsSpeakerCommand(SwerveDrive drive, ShooterSubsystem shooter) {
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
        ChassisSpeeds robotSpeeds = drive.getFieldRelativeSpeeds();

        Translation2d robotPos = robotPose.getTranslation();

        Translation2d target = shooter.getSpeakerPosition();
        double distToTarget = target.getDistance(robotPos);
        Rotation2d angleToTarget = target.minus(robotPos).getAngle();

        // Relative to the target
        Translation2d robotVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond).rotateBy(angleToTarget);

        double flyTime = NTData.SHOOTER_FLY_TIME.get();
        double missAmount = flyTime * robotVelocity.getY();

        double correctionRad = -Math.atan2(missAmount, distToTarget);
        // System.out.printf("FH: %.3f NH: %.3f D: %.3f FT: %.3f TV: %.3f C(d): %.3f\n", horizFlywheelVelocity, horizNoteVelocity, distToTarget, flightTime, tangentialVel, Math.toDegrees(correctionRad));

        double setpointAngle = MathUtil.wrap(angleToTarget.getRadians() + correctionRad + Math.toRadians(NTData.DRIVE_AIM_OFFSET.get()), -Math.PI, Math.PI);
        double currentAngle = MathUtil.wrap(robotPose.getRotation().getRadians(), -Math.PI, Math.PI);

        double max = NTData.DRIVE_AIM_MAX_TURN.get() * MathUtil.TAU;
        double output = pid.calculate(currentAngle, setpointAngle);
        errorRad = pid.getPositionError();
        output = MathUtil.clamp(output, -max, max);

        drive.turn(new SwerveDrive.TurnRequest(SwerveDrive.AIM_PRIORITY, new Rotation2d(output)));
    }

    public boolean isInTolerance(double tolRotations) {
        return errorRad < tolRotations * MathUtil.TAU;
//        Pose2d robotPose = drive.getEstimatedPose();
//        Translation2d robotPos = robotPose.getTranslation();
//        Translation2d target = targetSupplier.get();
//
//        double angleToTarget = target.minus(robotPos).getAngle().getRadians();
//        double currentAngle = MathUtil.wrap(robotPose.getRotation().getRadians(), -Math.PI, Math.PI);
//
//        return MathUtil.absDiffRad(currentAngle, angleToTarget) < tolRotations * MathUtil.TAU;
    }
}
