package com.swrobotics.robot.subsystems.limelightvision;

import com.swrobotics.lib.net.NTInteger;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.subsystems.tagtracker.TagTrackerInput;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.List;

public class LimelightVisionInput {
    private static final double XY_STD_DEV_COEFF = 0.01;
    private static final double THETA_STD_DEV_COEFF = 0.04;

    // Set to non-zero to override MegaTag version
    public static final NTInteger kMegaTagVersion = new NTInteger("MegaTag Version", 0);
    public static final NTInteger kMegaTagVersionOverride = new NTInteger("MegaTag Version Override", 0);

    private final String name;
    private double prevUpdateTimestamp;
    private boolean useMegaTag2;

    public LimelightVisionInput(String name) {
        this.name = name;
        prevUpdateTimestamp = Double.NaN;
        useMegaTag2 = false;
    }

    public void updateRobotState(double yawDegrees, ChassisSpeeds driveMotion) {
        // Assume robot isn't tipping over
        LimelightHelpers.SetRobotOrientation(name, yawDegrees, driveMotion.omegaRadiansPerSecond, 0, 0, 0, 0);

        // Use MT2 when moving, MT1 when still to readjust gyro
        useMegaTag2 = driveMotion.vxMetersPerSecond >= 0.2 || driveMotion.vyMetersPerSecond >= 0.2;
    }

    public void getNewUpdates(List<TagTrackerInput.VisionUpdate> updatesOut) {
        boolean useMegaTag2 = this.useMegaTag2;
        if (kMegaTagVersionOverride.get() != 0) {
            useMegaTag2 = kMegaTagVersionOverride.get() == 2;
        }
        kMegaTagVersion.set(useMegaTag2 ? 2 : 1);

        LimelightHelpers.PoseEstimate est;
        if (useMegaTag2) {
            est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        } else {
            est = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        }

        int tagCount = est.tagCount;
        if (tagCount <= 0)
            return;

        double timestamp = est.timestampSeconds;
        if (timestamp == prevUpdateTimestamp)
            return;
        prevUpdateTimestamp = timestamp;

        Pose2d robotPose = est.pose;
        double avgTagDist = est.avgTagDist;

        // Copied from TagTrackerInput
        double xyStdDev = XY_STD_DEV_COEFF * MathUtil.square(avgTagDist) / tagCount;
        double thetaStdDev = THETA_STD_DEV_COEFF * MathUtil.square(avgTagDist) / tagCount;

        // Don't trust MT2 theta at all, it's just gyro theta but with latency
        if (useMegaTag2)
            thetaStdDev = 99999999999999.0;
        updatesOut.add(new TagTrackerInput.VisionUpdate(
                timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
    }
}
