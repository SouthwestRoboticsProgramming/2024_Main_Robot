package com.swrobotics.robot.subsystems.music;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.swrobotics.robot.RobotContainer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public final class MusicSubsystem extends SubsystemBase {
    private static final String songsFolder = "music";

    public static List<String> getAvailableSongs() {
        File dir = new File(Filesystem.getDeployDirectory(), songsFolder);

        File[] files = dir.listFiles();
        List<String> out = new ArrayList<>();
        for (File file : files) {
            if (file.getName().endsWith(".chrp")) {
                out.add(file.getAbsolutePath());
            }
        }

        return out;
    }

    private final Orchestra orchestra;

    public MusicSubsystem(RobotContainer robot) {
        orchestra = new Orchestra();

        addInstrument(robot.drive.getDriveMotor(0));
        addInstrument(robot.drive.getDriveMotor(1));
        addInstrument(robot.drive.getDriveMotor(2));
        addInstrument(robot.drive.getDriveMotor(3));
        addInstrument(robot.drive.getTurnMotor(0));
        addInstrument(robot.drive.getTurnMotor(1));
        addInstrument(robot.drive.getTurnMotor(2));
        addInstrument(robot.drive.getTurnMotor(3));
        addInstrument(robot.climber.getLeftMotor());
        addInstrument(robot.climber.getRightMotor());
        addInstrument(robot.ampArm2.getMotor());
        addInstrument(robot.shooter.getLeftFlywheelMotor());
        addInstrument(robot.shooter.getRightFlywheelMotor());
        addInstrument(robot.shooter.getPivotMotor());
    }

    private void addInstrument(TalonFX fx) {
        AudioConfigs conf = new AudioConfigs();
        conf.BeepOnBoot = true;
        conf.BeepOnConfig = true;
        conf.AllowMusicDurDisable = true;
        fx.getConfigurator().apply(conf);
        
        orchestra.addInstrument(fx);
    }

    public void beginSong(String file) {
        orchestra.loadMusic(file);
        orchestra.play();
    }

    public boolean isSongPlaying() {
        return orchestra.isPlaying();
    }

    public void endSong() {
        orchestra.stop();
    }
}
