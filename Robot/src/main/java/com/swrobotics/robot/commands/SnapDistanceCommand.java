package com.swrobotics.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.swrobotics.lib.net.NTUtil;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.subsystems.speaker.ShooterSubsystem;
import com.swrobotics.robot.subsystems.speaker.aim.TableAimCalculator;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public final class SnapDistanceCommand extends Command {
    private final SwerveDrive drive;
    private final ShooterSubsystem shooter;

    private final PIDController pid;
    private final boolean wantMoveCloser;
    private double targetDistance;

    public SnapDistanceCommand(SwerveDrive drive, ShooterSubsystem shooter, boolean wantMoveCloser) {
        this.drive = drive;
        this.shooter = shooter;
        this.wantMoveCloser = wantMoveCloser;

        pid = NTUtil.tunablePID(NTData.DRIVE_SNAP_KP, NTData.DRIVE_SNAP_KD);
    }

    @Override
    public void initialize() {
        pid.reset();

        Pose2d robotPose = drive.getEstimatedPose();
        Translation2d target = shooter.getSpeakerPosition();
        Translation2d targetToRobot = robotPose.getTranslation().minus(target);

        double currentDist = targetToRobot.getNorm();
        targetDistance = TableAimCalculator.INSTANCE.getSnapDistance(currentDist, wantMoveCloser);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getEstimatedPose();

        Translation2d target = shooter.getSpeakerPosition();
        Translation2d targetToRobot = robotPose.getTranslation().minus(target);

        double currentDist = targetToRobot.getNorm();

        if (Math.abs(currentDist - targetDistance) < NTData.DRIVE_SNAP_DEADBAND.get())
            return;

        System.out.println("SNAP: " + currentDist + " -> " + targetDistance);
        double speed = pid.calculate(currentDist, targetDistance);
        double max = NTData.DRIVE_SNAP_MAX_SPEED.get();
        speed = MathUtil.clamp(speed, -max, max);

        Translation2d driveDemand = targetToRobot.div(currentDist).times(speed);
        drive.drive(new SwerveDrive.DriveRequest(
                SwerveDrive.SNAP_PRIORITY,
                drive.toRobotRelativeDrive(driveDemand),
                // Open loop since we're doing the feedback control here
                SwerveModule.DriveRequestType.OpenLoopVoltage
        ));
    }
}
