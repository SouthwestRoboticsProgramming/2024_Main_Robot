package com.swrobotics.robot.commands;

import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.NTData;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public final class RobotCommands {
    public static Command create(RobotContainer robot) {
        AimAtPointCommand aim = new AimAtPointCommand(robot.drive, robot.shooter::getSpeakerPosition);
        Command shootSeq = Commands.sequence(
                Commands.waitUntil(() -> aim.isInTolerance(NTData.DRIVE_AIM_TOLERANCE.get())),
                new ShootCommand(robot)
        );

        return new ParallelDeadlineGroup(shootSeq, aim);
    }
}
