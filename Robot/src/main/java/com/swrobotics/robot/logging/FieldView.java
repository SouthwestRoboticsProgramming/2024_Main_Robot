package com.swrobotics.robot.logging;

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

    public static void publish() {
        SmartDashboard.putData("Field View", field);
    }
}
