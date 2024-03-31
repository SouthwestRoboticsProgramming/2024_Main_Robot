package com.swrobotics.robot.commands;

import com.swrobotics.lib.net.NTEntry;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public final class AimTowardsLobCommand extends Command {
    private static final double OFFSET = -Math.atan2(64 - 17, (54*12 - 134) - 113);

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
        ChassisSpeeds robotSpeeds = drive.getFieldRelativeSpeeds();
        
        Translation2d target = shooter.getLobZonePosition();
        Rotation2d angleToTarget = target.minus(robotPos).getAngle();
        
        // Relative to the target
        Translation2d robotVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond).rotateBy(angleToTarget);

        AimCalculator.Aim currentAim = shooter.getTargetAim();
        double flyTime = getFlyTime(currentAim);
        double missAmount = flyTime * robotVelocity.getY();

        // For some reason it's different per alliance???
        NTEntry<Double> offset = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? NTData.SHOOTER_LOB_DRIVE_ANGLE_CORRECTION_BLUE
                : NTData.SHOOTER_LOB_DRIVE_ANGLE_CORRECTION_RED;

        double correctionRad = -Math.atan2(missAmount, currentAim.distanceToSpeaker()) + Math.toRadians(offset.get());

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

    private double getFlyTime(AimCalculator.Aim aim) {
        return Math.cos(aim.pivotAngle()) * aim.flywheelVelocity() / NTData.SHOOTER_LOB_POWER_COEFFICIENT.get() / 9.8;
    }
}
