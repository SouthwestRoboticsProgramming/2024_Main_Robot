package com.swrobotics.robot.commands;

import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.NTData;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public final class RobotCommands {
    public static Command aimAndShoot(RobotContainer robot, boolean waitForNote) {
        AimTowardsSpeakerCommand aim = new AimTowardsSpeakerCommand(robot.drive, robot.shooter);
        Command shootSeq = Commands.sequence(
                Commands.waitUntil(() -> (!waitForNote || robot.indexer.hasPiece())
                        && aim.isInTolerance(NTData.DRIVE_AIM_TOLERANCE.get())
                        && robot.shooter.isReadyToShoot())
                    .withTimeout(NTData.SHOOTER_AUTO_READY_TIMEOUT.get()),
                Commands.waitSeconds(NTData.SHOOTER_AUTO_AFTER_READY_DELAY.get()),
                new IndexerFeedCommand(robot.indexer)
        );

        return new ParallelDeadlineGroup(shootSeq, aim);
    }

    public static Command ejectHard(RobotContainer robot) {
        return Commands.sequence(
            Commands.waitUntil(() -> robot.shooter.isReadyToShoot())
                    .withTimeout(NTData.SHOOTER_AUTO_READY_TIMEOUT.get()),
                // Commands.waitSeconds(NTData.SHOOTER_AUTO_AFTER_READY_DELAY.get()),
                new IndexerFeedCommand(robot.indexer)
        );
    }
}
