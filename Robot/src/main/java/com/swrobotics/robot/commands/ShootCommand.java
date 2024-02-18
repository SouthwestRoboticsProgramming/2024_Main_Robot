package com.swrobotics.robot.commands;

import com.swrobotics.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(RobotContainer robot) {
        addCommands(
                Commands.waitUntil(robot.shooter::isReadyToShoot),
                Commands.waitSeconds(0.5), // Give flywheel a little more time
                new IndexerFeedCommand(robot.indexer)
        );
    }
}
