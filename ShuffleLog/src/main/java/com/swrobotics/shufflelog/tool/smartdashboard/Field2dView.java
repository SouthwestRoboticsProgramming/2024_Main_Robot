package com.swrobotics.shufflelog.tool.smartdashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;

import java.util.HashMap;
import java.util.Map;

public final class Field2dView {
    public final Map<String, Pose2d[]> poseSets;

    public Field2dView(NetworkTable table) {
        poseSets = new HashMap<>();

        for (String key : table.getKeys()) {
            if (key.startsWith("."))
                continue;

            double[] components = table.getEntry(key).getDoubleArray(new double[0]);
            Pose2d[] poses = new Pose2d[components.length / 3];
            for (int i = 0; i < poses.length; i++) {
                poses[i] = new Pose2d(
                        components[i * 3],
                        components[i * 3 + 1],
                        Rotation2d.fromDegrees(components[i * 3 + 2])
                );
            }

            poseSets.put(key, poses);
        }
    }
}
