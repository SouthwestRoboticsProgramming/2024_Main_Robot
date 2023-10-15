package com.swrobotics.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final double TRACKWIDTH_METERS =
            Units.inchesToMeters(18.5); // Configured for Nessie
    public static final double TRACKHEIGHT_METERS =
            Units.inchesToMeters(18.5); // Configured for Nessie

    public static final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    // Front left
                    new Translation2d(TRACKHEIGHT_METERS / 2.0, TRACKHEIGHT_METERS / 2.0),
                    // Front right
                    new Translation2d(TRACKHEIGHT_METERS / 2.0, -TRACKHEIGHT_METERS / 2.0),
                    // Back left
                    new Translation2d(-TRACKHEIGHT_METERS / 2.0, TRACKHEIGHT_METERS / 2.0),
                    // Back right
                    new Translation2d(-TRACKHEIGHT_METERS / 2.0, -TRACKHEIGHT_METERS / 2.0));
}
