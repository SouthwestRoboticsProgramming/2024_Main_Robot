package com.swrobotics.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// TODO: Bring in TagTracker estimation
public final class SwerveEstimator {
    private Pose2d currentPose;

    private final FieldObject2d poseVis;

    public SwerveEstimator() {
        currentPose = new Pose2d();

        Field2d field = new Field2d();
        poseVis = field.getObject("Pose Estimate");
        SmartDashboard.putData("Field", field);
    }

    public void resetPose(Pose2d knownPose) {
        currentPose = knownPose;
    }

    public void addDriveMovement(Twist2d twist) {
        currentPose = currentPose.exp(twist);
        poseVis.setPose(currentPose);
    }

    public Pose2d getEstimatedPose() {
        return currentPose;
    }
}
