package com.swrobotics.robot.subsystems.swerve;

import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTInteger;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.subsystems.tagtracker.CameraCaptureProperties;
import com.swrobotics.robot.subsystems.tagtracker.TagTrackerInput;
import com.swrobotics.robot.utils.GeometryUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public final class SwerveEstimator {
    //private static final double[] STATE_STD_DEVS = {0.003, 0.003, 0.0002};
    private static final double[] STATE_STD_DEVS = {0.005, 0.005, 0.001};
    private static final double HISTORY_TIME = 0.3;

    private static final double INITIAL_ANGLE_STDDEV = 0.2; // Really trust it for beginning of match

    private final TagTrackerInput tagTracker;

    private Pose2d basePose, latestPose;
    private final TreeMap<Double, PoseUpdate> updates = new TreeMap<>();
    private final Matrix<N3, N1> q, qWithMoreTrustAngle;

    private boolean ignoreVision;

    private final NTInteger numberOfVisionUpdates = new NTInteger("Debug/Number of Vision Updates", 0);
    private final NTBoolean seenWhereWeAre = new NTBoolean("Debug/Has Seen Where We Are", false);

    public SwerveEstimator(FieldInfo field) {
        double halfFrameL = 0.77 / 2;
        double halfFrameW = 0.695 / 2;

        CameraCaptureProperties captureProps = new CameraCaptureProperties()
                .setExposure(20)
                .setGain(5);

        tagTracker = new TagTrackerInput(
                field,

                new TagTrackerInput.CameraInfo(
                        "front",
                        (camPose) -> {
                            return camPose
                                    // Compensate for mounting angle
//                                    .transformBy(new Transform3d(new Translation3d(), new Rotation3d(Math.PI, Math.toRadians(90-67), 0)))
                                    .transformBy(new Transform3d(new Translation3d(), new Rotation3d(Math.PI, 0, 0)))
                                    .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, Math.toRadians(90-67), 0)))
                                    // Compensate for mounting position
                                    .transformBy(new Transform3d(new Translation3d(halfFrameL - 0.046, -halfFrameW + 0.12, 0.19).unaryMinus(), new Rotation3d()));
                        },
                        captureProps,
                        5), // meters

                new TagTrackerInput.CameraInfo(
                        "zoom",
                        (camPose) -> {
                            return camPose
                                    // Compensate for mounting angle
                                    .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, Math.toRadians(16.88), 0)))
                                    // Compensate for mounting position
                                    .transformBy(new Transform3d(new Translation3d(halfFrameL - 0.16, halfFrameW - 0.133, 0.39).unaryMinus(), new Rotation3d()));
                        },
                        captureProps,
                        Double.POSITIVE_INFINITY) // Trust at all distances
        );

        latestPose = new Pose2d();
        basePose = new Pose2d();

        q = new Matrix<>(Nat.N3(), Nat.N1());
        for (int i = 0; i < 3; i++) {
            q.set(i, 0, MathUtil.square(STATE_STD_DEVS[i]));
        }

        qWithMoreTrustAngle = new Matrix<>(Nat.N3(), Nat.N1());
        for (int i = 0; i < 3; i++) {
            qWithMoreTrustAngle.set(i, 0, MathUtil.square(i == 2 ? INITIAL_ANGLE_STDDEV : STATE_STD_DEVS[i]));
        }

        ignoreVision = false;
    }

    public void setIgnoreVision(boolean ignoreVision) {
        this.ignoreVision = ignoreVision;
    }

    public Pose2d getEstimatedPose() {
        return latestPose;
    }

    public void resetPose(Pose2d newPose) {
        basePose = newPose;
        updates.clear();
        update(false);
    }

    private double enabledTimestamp = Double.NaN;

    public void update(Twist2d driveTwist) {
        // Trust vision angle estimates for 5 seconds at start of teleop
        boolean trustTurnMore = false;
        if (DriverStation.isTeleopEnabled() && !DriverStation.isFMSAttached()) {
            if (Double.isNaN(enabledTimestamp)) {
                enabledTimestamp = Timer.getFPGATimestamp();
            }

            if (Timer.getFPGATimestamp() - enabledTimestamp < 5) {
                trustTurnMore = true;
            }
        } else {
            enabledTimestamp = Double.NaN;
        }

        // Sample vision data before updating drive so the drive is guaranteed
        // to be the most recent update
        List<TagTrackerInput.VisionUpdate> visionData = tagTracker.getNewUpdates();
        updates.put(Timer.getFPGATimestamp(), new PoseUpdate(driveTwist, new ArrayList<>()));

        numberOfVisionUpdates.set(numberOfVisionUpdates.get() + visionData.size());
        seenWhereWeAre.set(hasSeenWhereWeAre());

        List<Pose2d> tagPoses = new ArrayList<>();
        for (Pose3d tagPose3d : tagTracker.getEnvironment().getAllPoses()) {
            tagPoses.add(tagPose3d.toPose2d());
        }
        FieldView.aprilTagPoses.setPoses(tagPoses);

        if (ignoreVision) {
            update(false);
            return;
        }

        for (TagTrackerInput.VisionUpdate visionUpdate : visionData) {
            double timestamp = visionUpdate.timestamp;

            if (updates.containsKey(timestamp)) {
                List<TagTrackerInput.VisionUpdate> oldUpdates = updates.get(timestamp).visionUpdates;
                oldUpdates.add(visionUpdate);
                oldUpdates.sort(this::compareStdDevs);
            } else {
                Map.Entry<Double, PoseUpdate> prevUpdate = updates.floorEntry(timestamp);
                Map.Entry<Double, PoseUpdate> nextUpdate = updates.ceilingEntry(timestamp);

                if (prevUpdate == null || nextUpdate == null)
                    return;

                Twist2d prevToVisionTwist = GeometryUtil.multiplyTwist(
                        nextUpdate.getValue().twist,
                        MathUtil.percent(timestamp, prevUpdate.getKey(), nextUpdate.getKey()));
                Twist2d visionToNextTwist = GeometryUtil.multiplyTwist(
                        nextUpdate.getValue().twist,
                        (nextUpdate.getKey() - timestamp) / (nextUpdate.getKey() - prevUpdate.getKey()));

                List<TagTrackerInput.VisionUpdate> newVisionUpdates = new ArrayList<>();
                newVisionUpdates.add(visionUpdate);

                // Insert new update entry for this vision update
                updates.put(timestamp, new PoseUpdate(prevToVisionTwist, newVisionUpdates));

                // Overwrite nextUpdate with twist after this vision update
                updates.put(nextUpdate.getKey(), new PoseUpdate(visionToNextTwist, nextUpdate.getValue().visionUpdates));
            }
        }

        update(trustTurnMore);
    }

    private void update(boolean trustTurnMore) {
        Matrix<N3, N1> selectedQ = trustTurnMore ? qWithMoreTrustAngle : q;

        while (updates.size() > 1 && updates.firstKey() < Timer.getFPGATimestamp() - HISTORY_TIME) {
            Map.Entry<Double, PoseUpdate> update = updates.pollFirstEntry();
            basePose = update.getValue().apply(basePose, selectedQ);
        }

        latestPose = basePose;
        for (Map.Entry<Double, PoseUpdate> entry : updates.entrySet()) {
            latestPose = entry.getValue().apply(latestPose, selectedQ);
        }

        // Debug field stuffs
        FieldView.robotPose.setPose(latestPose);
        List<Pose2d> visionPoses = new ArrayList<>();
        for (PoseUpdate update : updates.values()) {
            for (TagTrackerInput.VisionUpdate visionUpdate : update.visionUpdates) {
                visionPoses.add(visionUpdate.estPose);
            }
        }
        FieldView.visionEstimates.setPoses(visionPoses);
    }

    private int compareStdDevs(TagTrackerInput.VisionUpdate u1, TagTrackerInput.VisionUpdate u2) {
        return -Double.compare(
                u1.stdDevs.get(0, 0) + u1.stdDevs.get(1, 0),
                u2.stdDevs.get(0, 0) + u2.stdDevs.get(1, 0)
        );
    }

    private static final class PoseUpdate {
        public final Twist2d twist;
        public final List<TagTrackerInput.VisionUpdate> visionUpdates;

        public PoseUpdate(Twist2d twist, List<TagTrackerInput.VisionUpdate> visionUpdates) {
            this.twist = twist;
            this.visionUpdates = visionUpdates;
        }

        public Pose2d apply(Pose2d prevPose, Matrix<N3, N1> q) {
            Pose2d pose = prevPose.exp(twist);

            for (TagTrackerInput.VisionUpdate visionUpdate : visionUpdates) {
                Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
                double[] r = new double[3];
                for (int i = 0; i < 3; i++)
                    r[i] = MathUtil.square(visionUpdate.stdDevs.get(i, 0));

                for (int row = 0; row < 3; row++) {
                    if (q.get(row, 0) == 0.0) {
                        visionK.set(row, row, 0.0);
                    } else {
                        double qRow0 = q.get(row, 0);
                        visionK.set(row, row, qRow0 / (qRow0 + Math.sqrt(qRow0 * r[row])));
                    }
                }

                Twist2d visionTwist = pose.log(visionUpdate.estPose);
                Matrix<N3, N1> twistMatrix = visionK.times(VecBuilder.fill(visionTwist.dx, visionTwist.dy, visionTwist.dtheta));

                pose = pose.exp(new Twist2d(twistMatrix.get(0, 0), twistMatrix.get(1, 0), twistMatrix.get(2, 0)));
            }

            return pose;
        }
    }

    public boolean hasSeenWhereWeAre() {
        // 1 second of frames from both cameras
        return numberOfVisionUpdates.get() > 100;
    }
}
