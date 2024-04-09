package com.swrobotics.robot.commands;

import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.subsystems.amp.AmpArm2Subsystem;
import com.swrobotics.robot.subsystems.speaker.IntakeSubsystem.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public final class RobotCommands {
    public static Command justShoot(RobotContainer robot) {
        return Commands.sequence(
            Commands.waitUntil(robot.shooter::isCalibrated),
            Commands.waitUntil(robot.shooter::isReadyToShoot)
                .withTimeout(NTData.SHOOTER_AUTO_READY_TIMEOUT.get()),
            Commands.waitSeconds(NTData.SHOOTER_AUTO_AFTER_READY_DELAY.get()),
            new IndexerFeedCommand(robot.indexer)
        );
    }

    public static Command aimAndShoot(RobotContainer robot, boolean waitForNote) {
        AimTowardsSpeakerCommand aim = new AimTowardsSpeakerCommand(robot.drive, robot.shooter);
        Command shootSeq = Commands.sequence(
                // Commands.runOnce(() -> robot.drive.setEstimatorIgnoreVision(false)),
                Commands.waitUntil(robot.shooter::isCalibrated),
                Commands.waitUntil(() -> (!waitForNote || robot.indexer.hasPiece())
                        && aim.isInTolerance(NTData.DRIVE_AIM_TOLERANCE.get())
                        && robot.shooter.isReadyToShoot())
                    .withTimeout(NTData.SHOOTER_AUTO_READY_TIMEOUT.get()),
                Commands.waitSeconds(NTData.SHOOTER_AUTO_AFTER_READY_DELAY.get()),
                new IndexerFeedCommand(robot.indexer)//,
                // Commands.runOnce(() -> robot.drive.setEstimatorIgnoreVision(true))
        );

        return new ParallelDeadlineGroup(shootSeq, aim);
    }

    public static Command shootQuick(RobotContainer robot) {
        return aimAndShoot(robot, false);
        // return Commands.sequence(
        //     Commands.runOnce(() -> robot.shooter.forcePivotCalibration(70)),
        //     Commands.waitUntil(() -> robot.shooter.isReadyToShoot()),
        //     new IndexerFeedCommand(robot.indexer)
        // );
    }

    public static Command ejectHard(RobotContainer robot) {
        return Commands.sequence(
            Commands.waitUntil(() -> robot.shooter.isReadyToShoot())
                    .withTimeout(NTData.SHOOTER_AUTO_READY_TIMEOUT.get()),
                // Commands.waitSeconds(NTData.SHOOTER_AUTO_AFTER_READY_DELAY.get()),
                new IndexerFeedCommand(robot.indexer)
        );
    }

    public static Command intakeUntilNote(RobotContainer robot) {
        return Commands.sequence(
            new IntakeSetCommand(robot.intake, State.INTAKE),
            Commands.waitUntil(() -> robot.indexer.hasPiece()).withTimeout(3.0),
            new IntakeSetCommand(robot.intake, State.OFF)
        );
    }

    public static Command flingNote(RobotContainer robot) {
        return Commands.sequence(
            Commands.runOnce(() -> robot.ampArm2.setPosition(AmpArm2Subsystem.Position.AMP)),
            Commands.waitSeconds(0.25),
            Commands.runOnce(() -> robot.ampArm2.setPosition(AmpArm2Subsystem.Position.RETRACT))
        );
    }
}
