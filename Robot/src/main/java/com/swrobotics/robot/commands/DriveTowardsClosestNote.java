package com.swrobotics.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveTowardsClosestNote {
    private static final double X = Units.inchesToMeters(250.5);
    public static final List<Translation2d> notePoses = List.of(
        new Translation2d(X, Units.inchesToMeters(29.64)),
        new Translation2d(X, Units.inchesToMeters(29.64 + 66 * 1)),
        new Translation2d(X, Units.inchesToMeters(29.64 + 66 * 2)),
        new Translation2d(X, Units.inchesToMeters(29.64 + 66 * 3)),
        new Translation2d(X, Units.inchesToMeters(29.64 + 66 * 4))
    );

    public static Command generate(Pose2d currentPose) {
        // Copy the list so that it can be temporarily edited
        List<Translation2d> notes = new ArrayList<>(notePoses);

        // Delete the closest note because we just missed it
        Translation2d closestNote = currentPose.getTranslation().nearest(notes);
        notes.remove(closestNote);
        Translation2d targetNote = currentPose.getTranslation().nearest(notes);

        // Set the angle to be going directly towards the target
        Rotation2d angle = targetNote.minus(currentPose.getTranslation()).getAngle();

        return AutoBuilder.pathfindToPose(new Pose2d(targetNote, angle), new PathConstraints(3.0, 3.0, 540, 540), 1.0);
    }
}
