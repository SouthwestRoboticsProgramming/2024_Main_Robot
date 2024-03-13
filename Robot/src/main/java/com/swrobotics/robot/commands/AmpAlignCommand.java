package com.swrobotics.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.swrobotics.lib.net.NTUtil;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.subsystems.speaker.ShooterSubsystem;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive.DriveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public final class AmpAlignCommand extends Command {
    private final SwerveDrive drive;

    private final PIDController turnPid;
    private double errorRad;

    public AmpAlignCommand(SwerveDrive drive, ShooterSubsystem shooter) {
        this.drive = drive;

        turnPid = NTUtil.tunablePID(NTData.DRIVE_AIM_KP, NTData.DRIVE_AIM_KD);
        turnPid.enableContinuousInput(-Math.PI, Math.PI);

        errorRad = Double.POSITIVE_INFINITY;
    }

    @Override
    public void initialize() {
        turnPid.reset();
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getEstimatedPose();

        Rotation2d targetAngle = Rotation2d.fromDegrees(90);
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            targetAngle.plus(Rotation2d.fromDegrees(180));
        }


        double setpointAngle = MathUtil.wrap(targetAngle.getRadians(), -Math.PI, Math.PI);
        double currentAngle = MathUtil.wrap(robotPose.getRotation().getRadians(), -Math.PI, Math.PI);

        double max = NTData.DRIVE_AIM_MAX_TURN.get() * MathUtil.TAU;
        double output = turnPid.calculate(currentAngle, setpointAngle);
        errorRad = turnPid.getPositionError();
        output = MathUtil.clamp(output, -max, max);

        drive.turn(new SwerveDrive.TurnRequest(SwerveDrive.SNAP_PRIORITY, new Rotation2d(output)));
        drive.drive(new DriveRequest(SwerveDrive.SNAP_PRIORITY, new Translation2d(), DriveRequestType.OpenLoopVoltage));
    }

    public boolean isInTolerance(double tolRotations) {
        return errorRad < tolRotations * MathUtil.TAU;
    }
}
