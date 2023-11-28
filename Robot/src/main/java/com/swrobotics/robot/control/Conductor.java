package com.swrobotics.robot.control;

import java.util.Optional;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

public class Conductor {
    public enum MusicTrack {
        RICK_ROLL("rick_roll"), // TODO: Actually make these .chrp files
        CARMELLA_DANSEN("carmella_dansen");

        private final String path;

        // Assign tracks to specific actions
        public static final MusicTrack STARTUP_JINGLE = RICK_ROLL;
        public static final MusicTrack DS_CONNECTED_JINGLE = null;
        public static final MusicTrack VICTORY_TRACK = CARMELLA_DANSEN;

        MusicTrack(String name) {
            path = "music/" + name + ".chrp";
        }

        public String getPath() {
            return path;
        }
    }

    private static final Orchestra orchestra = new Orchestra();

    public static StatusCode loadMusic(String path) {
        return orchestra.loadMusic(path);
    }

    public static StatusCode playTrack(MusicTrack track) {
        try {  
            return loadMusic(track.getPath());
        } catch (Exception e) {
            return StatusCode.MusicFileInvalid;
        }
    }

    public static StatusCode play() {
        return orchestra.play();
    }

    public static StatusCode pause() {
        return orchestra.pause();
    }

    public static StatusCode stop() {
        return orchestra.stop();
    }

    public static boolean isPlaying() {
        return orchestra.isPlaying();
    }

    public static double getCurrentTrackTime() {
        return orchestra.getCurrentTime();
    }

    public static StatusCode addInstrument(ParentDevice instrument) {
        return orchestra.addInstrument(instrument);
    }

    public static StatusCode addInstrument(ParentDevice instrument, int trackNumber) {
        return orchestra.addInstrument(instrument, trackNumber);
    }

    public static StatusCode clearInstruments() {
        return orchestra.clearInstruments();
    }


}
