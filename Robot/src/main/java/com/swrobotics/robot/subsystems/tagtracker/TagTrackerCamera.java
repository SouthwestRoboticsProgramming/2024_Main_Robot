package com.swrobotics.robot.subsystems.tagtracker;

import com.swrobotics.robot.subsystems.tagtracker.io.NTCameraIO;
import com.swrobotics.robot.subsystems.tagtracker.io.TagTrackerCameraIO;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

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

        public static EstimateInput decode(long ntTimestamp, double[] data) {
            double timestamp = ntTimestamp / 1_000_000.0;

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

    private final String name;
//    private final Transform3d toRobotTransform;
    private final Function<Pose3d, Pose3d> cameraToRobot;

    private final TagTrackerCameraIO io;
    private final TagTrackerCameraIO.Inputs inputs;

    public TagTrackerCamera(String name, NetworkTable table, Function<Pose3d, Pose3d> cameraToRobot, CameraCaptureProperties captureProps) {
        this.name = name;
//        this.toRobotTransform = toRobotTransform;
        this.cameraToRobot = cameraToRobot;

        io = new NTCameraIO(table);
        inputs = new TagTrackerCameraIO.Inputs();

        io.setCaptureProperties(captureProps);
    }

    public Function<Pose3d, Pose3d> getToRobotTransform() {
        return cameraToRobot;
    }

    public List<EstimateInput> getEstimates() {
        io.updateInputs(inputs);
        Logger.processInputs("TagTracker/Camera/" + name, inputs);

        List<EstimateInput> estimates = new ArrayList<>();

        for (int i = 0; i < inputs.timestamps.length; i++) {
            long timestamp = inputs.timestamps[i];
            double[] data = inputs.framePackedData[i];

            EstimateInput input = EstimateInput.decode(timestamp, data);
            if (input != null)
                estimates.add(input);
        }

        return estimates;
    }
}
