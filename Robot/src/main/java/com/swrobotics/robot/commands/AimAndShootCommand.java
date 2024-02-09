package com.swrobotics.robot.commands;

import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.NTData;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public final class AimAndShootCommand extends ParallelCommandGroup {
    public AimAndShootCommand(RobotContainer robot) {
        AimAtPointCommand aim = new AimAtPointCommand(robot.drive, robot.shooter::getSpeakerPosition);

        addCommands(
                aim,
                Commands.sequence(
                        Commands.waitUntil(() -> aim.isInTolerance(NTData.DRIVE_AIM_TOLERANCE.get())),
                        new ShootCommand(robot)
                )
        );
    }
}
