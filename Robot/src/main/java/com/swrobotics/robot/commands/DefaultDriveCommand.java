package com.swrobotics.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.control.ControlBoard;

import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultDriveCommand extends Command {
    private final SwerveDrive drive;
    private final ControlBoard input;

    public DefaultDriveCommand(SwerveDrive drive, ControlBoard controlBoard) {
        this.drive = drive;
        input = controlBoard;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        Translation2d rawTranslation = input.getDriveTranslation();
        double rawRotation = input.getDriveRotation();

        // Scale inputs
        Translation2d translation = rawTranslation.times(SwerveDrive.MAX_LINEAR_SPEED);
        Rotation2d rotation = new Rotation2d(MathUtil.TAU * rawRotation);

        drive.drive(new SwerveDrive.DriveRequest(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation.getRadians(),
                        drive.getEstimatedPose().getRotation()),
                SwerveModule.DriveRequestType.OpenLoopVoltage));
    }
}
