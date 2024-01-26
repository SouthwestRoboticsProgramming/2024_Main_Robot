package com.swrobotics.robot.subsystems.music;

import com.ctre.phoenix6.Orchestra;
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

        orchestra.addInstrument(robot.drive.modules[0].getDriveMotor());
        orchestra.addInstrument(robot.drive.modules[1].getDriveMotor());
        orchestra.addInstrument(robot.drive.modules[2].getDriveMotor());
        orchestra.addInstrument(robot.drive.modules[3].getDriveMotor());
        orchestra.addInstrument(robot.drive.modules[0].getSteerMotor());
        orchestra.addInstrument(robot.drive.modules[1].getSteerMotor());
        orchestra.addInstrument(robot.drive.modules[2].getSteerMotor());
        orchestra.addInstrument(robot.drive.modules[3].getSteerMotor());
        orchestra.addInstrument(robot.climber.leftArm.motor);
        orchestra.addInstrument(robot.climber.rightArm.motor);
        orchestra.addInstrument(robot.ampArm.motor);
        orchestra.addInstrument(robot.shooter.flywheelMotor1);
        orchestra.addInstrument(robot.shooter.flywheelMotor2);
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
