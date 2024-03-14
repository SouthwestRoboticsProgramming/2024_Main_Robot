package com.swrobotics.robot.commands;

import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.subsystems.speaker.IndexerSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public final class IndexerFeedCommand extends Command {
    private final IndexerSubsystem indexer;
    private final Timer timer;

    public IndexerFeedCommand(IndexerSubsystem indexer) {
        this.indexer = indexer;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        indexer.setFeedToShooter(true);
        SimView.setShooting(true);
        if (!indexer.hasPiece()) {
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setFeedToShooter(false);
        SimView.setShooting(false);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(NTData.INDEXER_FEED_TIME.get());
    }
}
