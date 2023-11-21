package com.swrobotics.robot.commands;

import com.swrobotics.mathlib.Angle;
import com.swrobotics.mathlib.CWAngle;
import com.swrobotics.mathlib.Vec2d;
import com.swrobotics.robot.control.ControlBoard;

import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
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
        Vec2d rawTranslation = input.getDriveTranslation();
        double rawRotation = input.getDriveRotation();

//        System.out.println("Running");

        // Scale inputs
        Vec2d translation = rawTranslation.mul(4.11);
        Angle rotation =
                CWAngle.rad(2 * Math.PI).mul(rawRotation);

        drive.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.x,
                        translation.y,
                        rotation.ccw().rad(),
                        drive.getEstimatedPose().getRotation()));
    }
}
