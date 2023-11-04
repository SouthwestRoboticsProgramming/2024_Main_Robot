package com.swrobotics.robot.subsystems.tagtracker;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

public final class TagTrackerEnvironment {
    private final DoubleArraySubscriber sub;
    private final Map<Integer, Pose3d> poses;
    private long lastChange;

    public TagTrackerEnvironment(DoubleArrayTopic topic) {
        sub = topic.subscribe(new double[0]);
        poses = new HashMap<>();
        lastChange = Long.MAX_VALUE;
    }

    // Returns pose if tag exists, else null
    public Pose3d getPose(int tagId) {
        return poses.get(tagId);
    }

    public Collection<Pose3d> getAllPoses() {
        return poses.values();
    }

    public void update() {
        long timestamp = sub.getLastChange();
        if (timestamp == lastChange)
            return;
        lastChange = timestamp;

        double[] data = sub.get();
        poses.clear();
        for (int i = 0; i < data.length; i += 8) {
            int tagId = (int) data[i];

            double tx = data[i + 1];
            double ty = data[i + 2];
            double tz = data[i + 3];
            double qw = data[i + 4];
            double qx = data[i + 5];
            double qy = data[i + 6];
            double qz = data[i + 7];

            Translation3d translation = new Translation3d(tx, ty, tz);
            Rotation3d rotation = new Rotation3d(new Quaternion(qw, qx, qy, qz));
            Pose3d pose = new Pose3d(translation, rotation);

            poses.put(tagId, pose);
        }
    }
}
