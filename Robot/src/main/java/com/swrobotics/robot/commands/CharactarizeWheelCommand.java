package com.swrobotics.robot.commands;

import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import com.swrobotics.lib.net.NTDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class CharactarizeWheelCommand extends Command {
    private final SwerveDrive drive;
    private SwerveModulePosition[] startingPositions;
    private double lastGyroRad;
    private double gyroAccumulatorRad;

    public CharactarizeWheelCommand(SwerveDrive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        startingPositions = drive.getCurrentModulePositions(true);
        gyroAccumulatorRad = 0;
        lastGyroRad = drive.getRawGyroRotation().getRadians();
    }

    NTDouble acc = new NTDouble("Drive/Wheel Radius acc", 0);
    
    @Override
    public void execute() {
        double currentGyroRad = drive.getRawGyroRotation().getRadians();
        gyroAccumulatorRad += MathUtil.angleModulus(currentGyroRad - lastGyroRad);
        lastGyroRad = currentGyroRad;
	acc.set(gyroAccumulatorRad);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveModulePosition[] endingPositions = drive.getCurrentModulePositions(true);
        double averageWheelDisplacement = 0;

        for (int i = 0; i < 4; i++) {
            averageWheelDisplacement += Math.abs(endingPositions[i].distanceMeters - startingPositions[i].distanceMeters);
        }
        averageWheelDisplacement /= 4;

        double effectiveWheelRadius = (gyroAccumulatorRad * SwerveDrive.DRiVE_RADIUS) / averageWheelDisplacement;
	new NTDouble("Drive/Wheel radius result", 0).set(effectiveWheelRadius);
        System.out.println("Effective Wheel Radius: " + Units.metersToInches(effectiveWheelRadius) + " inches");
    }
}
