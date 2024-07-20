package com.swrobotics.robot.commands;

import java.util.ArrayList;
import java.util.List;

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

    // public static Command aimAndShoot(RobotContainer robot, boolean waitForNote, boolean useVision) {
    //     AimTowardsSpeakerCommand aim = new AimTowardsSpeakerCommand(robot.drive, robot.shooter);
    //     List<Command> commands = new ArrayList<>();
    //     commands.addAll(List.of(
    //             Commands.waitUntil(robot.shooter::isCalibrated),
    //             Commands.waitUntil(() -> (!waitForNote || robot.indexer.hasPiece())
    //                     && aim.isInTolerance(NTData.DRIVE_AIM_TOLERANCE.get())
    //                     && robot.shooter.isReadyToShoot())
    //                 .withTimeout(NTData.SHOOTER_AUTO_READY_TIMEOUT.get()),
    //             Commands.waitSeconds(NTData.SHOOTER_AUTO_AFTER_READY_DELAY.get()),
    //             new IndexerFeedCommand(robot.indexer)
    //     ));

    //     if (useVision) {
    //         commands.add(0, Commands.runOnce(() -> robot.drive.setEstimatorIgnoreVision(false)));
    //         commands.add(Commands.runOnce(() -> robot.drive.setEstimatorIgnoreVision(true)));
    //     }

    //     return new ParallelDeadlineGroup(Commands.sequence(commands.toArray(new Command[0])), aim);
    // }

    public static Command aimAndShoot(RobotContainer robot, boolean waitForNote, boolean useVision) {
        Command cmd = robot.shooter.getSpeakerSnapCommand();
        // .andThen(Commands.waitUntil(robot.shooter::isCalibrated),
        //          Commands.waitUntil(() -> (!waitForNote || robot.indexer.hasPiece())
        //                 && robot.shooter.isReadyToShoot())
        //             .withTimeout(NTData.SHOOTER_AUTO_READY_TIMEOUT.get()),
        //          Commands.waitSeconds(NTData.SHOOTER_AUTO_AFTER_READY_DELAY.get()));
        
        // if (useVision) {
        //     cmd = cmd.beforeStarting(Commands.runOnce(() -> robot.drive.setEstimatorIgnoreVision(false)));
        //     cmd = cmd.finallyDo(() -> robot.drive.setEstimatorIgnoreVision(true));
        // }

        cmd = cmd.andThen(new IndexerFeedCommand(robot.indexer));
        return cmd;
    }

    public static Command shootQuick(RobotContainer robot) {
        return aimAndShoot(robot, false, true);
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

    public static Command intakeUntilNote(RobotContainer robot, boolean reindex) {
        return Commands.sequence(
                Commands.runOnce(() -> robot.indexer.setAutoReindexEnable(reindex)),
            new IntakeSetCommand(robot.intake, State.INTAKE),
            Commands.waitUntil(() -> robot.indexer.hasPiece()).withTimeout(3.0),
            new IntakeSetCommand(robot.intake, State.OFF),
                Commands.runOnce(() -> robot.indexer.setAutoReindexEnable(false))
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
