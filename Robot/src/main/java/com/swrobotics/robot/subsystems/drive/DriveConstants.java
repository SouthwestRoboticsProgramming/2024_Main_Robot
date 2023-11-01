package com.swrobotics.robot.subsystems.drive;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final double TRACKWIDTH_METERS =
            Units.inchesToMeters(18.5); // Configured for Nessie
    public static final double TRACKHEIGHT_METERS =
            Units.inchesToMeters(18.5); // Configured for Nessie

    public static final Translation2d[] modulePositions = {
      // Front left
      new Translation2d(TRACKHEIGHT_METERS / 2.0, TRACKHEIGHT_METERS / 2.0),
      // Front right
      new Translation2d(TRACKHEIGHT_METERS / 2.0, -TRACKHEIGHT_METERS / 2.0),
      // Back left
      new Translation2d(-TRACKHEIGHT_METERS / 2.0, TRACKHEIGHT_METERS / 2.0),
      // Back right
      new Translation2d(-TRACKHEIGHT_METERS / 2.0, -TRACKHEIGHT_METERS / 2.0)      
    };

    public static final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(modulePositions);

    /** Meters per second */
    public static final double MAX_SPEED = Units.feetToMeters(18.0); // FIXME: Test empirically
    /** Radians per second */
    public static final double MAX_ANGULAR_SPEED = MAX_SPEED / Math.hypot(TRACKWIDTH_METERS / 2.0, TRACKHEIGHT_METERS / 2.0);
}
