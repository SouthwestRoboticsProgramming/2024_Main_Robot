package com.swrobotics.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive.DriveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveIntoWallCommand extends Command {
    private final SwerveDrive drive;
    private int count;
    
    public DriveIntoWallCommand(SwerveDrive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        count = 0;
    }

    @Override
    public void execute() {
        count++;
        drive.drive(new DriveRequest(SwerveDrive.SNAP_PRIORITY, new Translation2d(0.25, 0), DriveRequestType.OpenLoopVoltage));
    }

    @Override
    public boolean isFinished() {
        return count > 15 && drive.getRobotRelativeSpeeds().vxMetersPerSecond < 0.1;
    }
}
