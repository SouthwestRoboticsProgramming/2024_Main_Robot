package com.swrobotics.robot.logging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class FieldView {
    private static final Field2d field = new Field2d();

    public static final FieldObject2d robotPose = field.getRobotObject();
    public static final FieldObject2d visionEstimates = field.getObject("Vision estimates");
    public static final FieldObject2d aprilTagPoses = field.getObject("AprilTag poses");
    public static final FieldObject2d pathPlannerPath = field.getObject("PathPlanner path");
    public static final FieldObject2d pathPlannerSetpoint = field.getObject("PathPlanner setpoint");

    public static final FieldObject2d startingPosition = field.getObject("Preload Position");
    static {
        startingPosition.setPose(-1, 0, new Rotation2d());
    }

    public static final FieldObject2d lobZone = field.getObject("Lob Zone");
    static {
        lobZone.setPose(1.3, 7.1, new Rotation2d());
    }

    public static void publish() {
        SmartDashboard.putData("Field View", field);
    }
}
