package com.swrobotics.robot.commands;

import com.swrobotics.robot.subsystems.music.MusicSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * ONLY works in disabled
 */
public final class PlaySongCommand extends Command {
    private final MusicSubsystem music;
    private final String song;

    public PlaySongCommand(MusicSubsystem music, String song) {
        this.music = music;
        this.song = song;
        addRequirements(music);
    }

    @Override
    public void initialize() {
        music.beginSong(song);
        System.out.println("Song is GO");
    }

    @Override
    public void end(boolean interrupted) {
        music.endSong();
    }

    @Override
    public boolean isFinished() {
        return !music.isSongPlaying();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
