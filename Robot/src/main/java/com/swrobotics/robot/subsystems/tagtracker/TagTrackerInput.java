package com.swrobotics.robot.subsystems.tagtracker;

import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.utils.GeometryUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public final class TagTrackerInput {
    public static final class CameraInfo {
        public final String name;
        public final Function<Pose3d, Pose3d> cameraToRobot;
//        public final Pose3d robotRelPose;

        public CameraInfo(String name, Function<Pose3d, Pose3d> cameraToRobot) {
            this.name = name;
            this.cameraToRobot = cameraToRobot;
//            this.robotRelPose = robotRelPose;
        }
    }

    public static final class VisionUpdate {
        public final double timestamp;
        public final Pose2d estPose;
        public final Vector<N3> stdDevs;

        public VisionUpdate(double timestamp, Pose2d estPose, Vector<N3> stdDevs) {
            this.timestamp = timestamp;
            this.estPose = estPose;
            this.stdDevs = stdDevs;
        }
    }

    private static final double AMBIGUITY_THRESHOLD = 0.9;
    private static final double FIELD_BORDER_MARGIN = 0.5;
    private static final double Z_MARGIN = 0.75;
    private static final double XY_STD_DEV_COEFF = 0.01;
    private static final double THETA_STD_DEV_COEFF = 0.01;

    private final FieldInfo field;
    private final TagTrackerEnvironment environment;
    private final TagTrackerCamera[] cameras;

    public TagTrackerInput(FieldInfo field, CameraInfo... infos) {
        this.field = field;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("TagTracker");
        environment = new TagTrackerEnvironment(table.getDoubleArrayTopic("environment"));

        NetworkTable camerasTable = table.getSubTable("Cameras");
        cameras = new TagTrackerCamera[infos.length];
        for (int i = 0; i < infos.length; i++) {
            CameraInfo info = infos[i];
            cameras[i] = new TagTrackerCamera(
                    info.name,
                    camerasTable.getSubTable(info.name),
                    info.cameraToRobot);
        }
    }

    private boolean outOfRange(double val, double min, double max) {
        return val < min || val > max;
    }

    public List<VisionUpdate> getNewUpdates() {
        environment.update();

        List<VisionUpdate> updates = new ArrayList<>();
        for (TagTrackerCamera camera : cameras) {
            for (TagTrackerCamera.EstimateInput input : camera.getEstimates()) {
                Pose3d cameraPose = null;
                Pose3d robotPose3d = null;
                if (input.poseB == null) {
                    // Only one pose available, camera sees multiple tags
                    cameraPose = input.poseA.pose;
                    robotPose3d = camera.getToRobotTransform().apply(cameraPose);
//                    robotPose3d = cameraPose.transformBy(camera.getToRobotTransform());
                } else {
                    // Two poses available (one tag), choose the better one.
                    // Pose is only chosen if it's significantly better than the
                    // other. This ignores tags where the two errors are close to
                    // each other (i.e. facing the tag straight on)
                    double errA = input.poseA.error;
                    double errB = input.poseB.error;
                    if (errA < errB * AMBIGUITY_THRESHOLD)
                        cameraPose = input.poseA.pose;
                    else if (errB < errA * AMBIGUITY_THRESHOLD)
                        cameraPose = input.poseB.pose;

                    if (cameraPose != null)
                        robotPose3d = camera.getToRobotTransform().apply(cameraPose);
//                        robotPose3d = cameraPose.transformBy(camera.getToRobotTransform());
                }

                // Skip frame if no pose was good
                if (cameraPose == null)
                    continue;


                // Skip frame if outside field
                if (outOfRange(robotPose3d.getX(), -FIELD_BORDER_MARGIN, field.getWidth() + FIELD_BORDER_MARGIN)
                    || outOfRange(robotPose3d.getY(), -FIELD_BORDER_MARGIN, field.getHeight() + FIELD_BORDER_MARGIN)
                    || outOfRange(robotPose3d.getZ(), -Z_MARGIN, Z_MARGIN)) {
                    continue;
                }

                Pose2d robotPose = robotPose3d.toPose2d();

                // Calculate the average distance to all the tags that are visible
                double totalTagDist = 0;
                int tagCount = 0;
                for (int tagId : input.visibleTagIds) {
                    Pose3d tagPose = environment.getPose(tagId);
                    if (tagPose != null) {
                        totalTagDist += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
                        tagCount++;
                    }
                }
                double avgTagDist = totalTagDist / tagCount;

                // Trust farther away tags less and frames with more tags more
                double xyStdDev = XY_STD_DEV_COEFF * MathUtil.square(avgTagDist) / tagCount;
                double thetaStdDev = THETA_STD_DEV_COEFF * MathUtil.square(avgTagDist) / tagCount;
                updates.add(new VisionUpdate(
                        input.timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
            }
        }

        return updates;
    }

    public TagTrackerEnvironment getEnvironment() {
        return environment;
    }
}
