package com.swrobotics.robot.subsystems.tagtracker;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class TagTrackerCamera {
    public static final class PoseEstimate {
        public final double error;
        public final Pose3d pose;

        public PoseEstimate(double[] data, int i) {
            error = data[i];

            double tx = data[i + 1];
            double ty = data[i + 2];
            double tz = data[i + 3];
            Translation3d translation = new Translation3d(tx, ty, tz);

            double qw = data[i + 4];
            double qx = data[i + 5];
            double qy = data[i + 6];
            double qz = data[i + 7];
            Rotation3d rotation = new Rotation3d(new Quaternion(qw, qx, qy, qz));

            pose = new Pose3d(translation, rotation);
        }

        @Override
        public String toString() {
            return "PoseEstimate{" +
                    "error=" + error +
                    ", pose=" + pose +
                    '}';
        }
    }

    public static final class EstimateInput {
        public final double timestamp;
        public final PoseEstimate poseA;
        public final PoseEstimate poseB;
        public final int[] visibleTagIds;

        public static EstimateInput decode(TimestampedDoubleArray dataEntry) {
            double timestamp = dataEntry.timestamp / 1_000_000.0;
            double[] data = dataEntry.value;

            int count = (int) data[0];
            if (count == 0)
                return null;

            PoseEstimate poseA = new PoseEstimate(data, 1);
            PoseEstimate poseB = count == 2 ? new PoseEstimate(data, 9) : null;

            int tagsOffset = count == 2 ? 17 : 9;
            int[] visibleTagIds = new int[data.length - tagsOffset - 1];
            for (int i = tagsOffset; i < data.length - 1; i++) {
                visibleTagIds[i - tagsOffset] = (int) data[i];
            }

            double timeOffset = data[data.length - 1];
            return new EstimateInput(timestamp - timeOffset, poseA, poseB, visibleTagIds);
        }

        private EstimateInput(double timestamp, PoseEstimate poseA, PoseEstimate poseB, int[] visibleTagIds) {
            this.timestamp = timestamp;
            this.poseA = poseA;
            this.poseB = poseB;
            this.visibleTagIds = visibleTagIds;
        }

        @Override
        public String toString() {
            return "EstimateInput{" +
                    "timestamp=" + timestamp +
                    ", poseA=" + poseA +
                    ", poseB=" + poseB +
                    ", visibleTagIds=" + Arrays.toString(visibleTagIds) +
                    '}';
        }
    }

    private final Transform3d toRobotTransform;
    private final DoubleArraySubscriber sub;

    public TagTrackerCamera(NetworkTable table, Transform3d toRobotTransform) {
        this.toRobotTransform = toRobotTransform;
        sub = table.getDoubleArrayTopic("poses").subscribe(new double[] {0}, PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
    }

    public Transform3d getToRobotTransform() {
        return toRobotTransform;
    }

    public List<EstimateInput> getInputs() {
        TimestampedDoubleArray[] data = sub.readQueue();
        List<EstimateInput> inputs = new ArrayList<>();
        for (TimestampedDoubleArray arr : data) {
            EstimateInput input = EstimateInput.decode(arr);
            if (input != null)
                inputs.add(input);
        }
        return inputs;
    }
}
