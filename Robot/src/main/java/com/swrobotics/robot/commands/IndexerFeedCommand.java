package com.swrobotics.robot.commands;

import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.subsystems.speaker.IndexerSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

@Deprecated
public final class IndexerFeedCommand extends Command {
    private final IndexerSubsystem indexer;
    private final Timer timer;
    private boolean hasStartedTimer;

    public IndexerFeedCommand(IndexerSubsystem indexer) {
        this.indexer = indexer;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.stop();
        hasStartedTimer = false;
    }

    @Override
    public void execute() {
        indexer.setFeedToShooter(true);
        if (!indexer.hasPiece() && !hasStartedTimer) {
            timer.start();
            hasStartedTimer = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setFeedToShooter(false);
    }

    @Override
    public boolean isFinished() {
        return hasStartedTimer && timer.hasElapsed(NTData.INDEXER_FEED_ADDITIONAL_TIME.get());
    }
}
