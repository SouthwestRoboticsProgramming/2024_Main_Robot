package com.swrobotics.robot.control;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ChoreoBuilder {
    private static ChoreoControlFunction controller;
    private static Supplier<Pose2d> pose;
    private static Consumer<ChassisSpeeds> chassis;
    private static boolean useAlliance;
    private static Subsystem[] reqiredSubsystems;

    public static void configure(PIDConstants translationConstants, PIDConstants rotationConstants, Supplier<Pose2d> poseSupplier, Consumer<ChassisSpeeds> chassisConsumer, boolean useAllianceColor, Subsystem... requirements) {
        controller = Choreo.choreoSwerveController(
            new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD),
            new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD),
            new PIDController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD));
        
        pose = poseSupplier;
        chassis = chassisConsumer;
        useAlliance = useAllianceColor;
        reqiredSubsystems = requirements;
    }

    public static Command getPath(String name) {
        return Choreo.choreoSwerveCommand(
            Choreo.getTrajectory(name), pose, controller, chassis, useAlliance, reqiredSubsystems);
    }
}
