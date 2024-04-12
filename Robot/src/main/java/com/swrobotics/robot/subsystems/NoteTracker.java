package com.swrobotics.robot.subsystems;

import java.util.Optional;

import com.swrobotics.robot.utils.LimelightHelpers;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class NoteTracker {

    private static final double MOUNTING_HEIGHT_METERS = 1; // FIXME: Measure
    private static final double PITCH = -10;
    private static final double YAW = -20;

    public static Optional<Double> getDistanceToNote() {
        if (LimelightHelpers.getLatestResults("").targetingResults.valid) {
            return Optional.of(
                (Math.tan(Math.toRadians(LimelightHelpers.getTY("")) + PITCH)) * MOUNTING_HEIGHT_METERS);
        }
        return Optional.empty();
    }

    public static Optional<Rotation2d> getAngleToNote() {
        // if (RobotBase.isSimulation()) {
        //     return Optional.of(Rotation2d.fromDegrees(90));
        // }

        if (LimelightHelpers.getLatestResults("").targetingResults.valid) {
            return Optional.ofNullable(Rotation2d.fromDegrees(LimelightHelpers.getTX("") + YAW));
        }

        return Optional.empty();
    }
}
