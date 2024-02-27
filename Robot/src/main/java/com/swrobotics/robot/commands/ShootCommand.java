package com.swrobotics.robot.commands;

import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.NTData;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(RobotContainer robot) {
        addCommands(
                Commands.waitUntil(robot.shooter::isReadyToShoot)
                        .withTimeout(NTData.SHOOTER_AUTO_READY_TIMEOUT.get()),
                Commands.waitSeconds(NTData.SHOOTER_AUTO_AFTER_READY_DELAY.get()), // Give flywheel a little more time
                new IndexerFeedCommand(robot.indexer)
        );
    }
}
