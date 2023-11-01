package com.swrobotics.robot.commands;

import com.swrobotics.mathlib.Angle;
import com.swrobotics.mathlib.CWAngle;
import com.swrobotics.mathlib.Vec2d;
import com.swrobotics.robot.control.ControlBoard;
import com.swrobotics.robot.subsystems.drive.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultDriveCommand extends CommandBase {
    private final Drive drive;
    private final ControlBoard input;

    public DefaultDriveCommand(Drive drive, ControlBoard controlBoard) {
        this.drive = drive;
        input = controlBoard;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        Vec2d rawTranslation = input.getDriveTranslation();
        double rawRotation = input.getDriveRotation();

        // Scale inputs
        Vec2d translation = rawTranslation.mul(Units.feetToMeters(18.0));

        Angle rotation = CWAngle.rad(3).mul(rawRotation);
        
        drive.setChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.x,
                        translation.y,
                        rotation.ccw().rad(),
                        drive.getPose().getRotation()));
    }
}
