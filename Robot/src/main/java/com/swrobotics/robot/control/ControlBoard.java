package com.swrobotics.robot.control;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.swrobotics.lib.input.XboxController;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.mathlib.Vec2d;
import com.swrobotics.robot.RobotContainer;

import com.swrobotics.robot.subsystems.swerve.pathfinding.CircleShape;
import com.swrobotics.robot.subsystems.swerve.pathfinding.RectangleShape;
import com.swrobotics.robot.subsystems.swerve.pathfinding.Shape;
import com.swrobotics.robot.subsystems.swerve.pathfinding.ThetaStarPathfinder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;
import java.util.List;

public class ControlBoard {
    private static final double DEADBAND = 0.15;

    /** Manages demands to go to an exact angle demanded by the driver */
    public enum SwerveCardinal {
        NONE(0),
        FRONT(0),
        LEFT(90),
        RIGHT(-90),
        BACk(180);

        public final double degrees;

        SwerveCardinal(double degrees) {
            this.degrees = degrees;
        }
    }

    public final XboxController driver;
    public final XboxController operator;

    public ControlBoard(RobotContainer robot) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            3.0, 4.0, 
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        Command pathfindingCommand = AutoBuilder.pathfindToPose(new Pose2d(7.5, 4.5, new Rotation2d()), constraints);
        
        driver = new XboxController(0, DEADBAND);
        operator = new XboxController(1, DEADBAND);

        driver.a.onFalling(pathfindingCommand);
        driver.b.onFalling(() -> {
            List<Shape> shapes = new ArrayList<>();

            // Generate some random shapes
            for (int i = 0; i < 5; i++) {
                double cx = Math.random() * 16;
                double cy = Math.random() * 8;

                if (Math.random() > 0.5) {
                    double w = Math.random() * 3;
                    double h = Math.random() * 3;

                    shapes.add(new RectangleShape(cx, cy, w, h, new Rotation2d(Math.random() * Math.PI), false));
                } else {
                    double rad = Math.random() * 2;
                    shapes.add(new CircleShape(cx, cy, rad));
                }
            }

//            Pathfinding.setDynamicObstacles(rects, robot.drive.getEstimatedPose().getTranslation());
            ThetaStarPathfinder.getInstance().setDynamicShapes(shapes, robot.drive.getEstimatedPose().getTranslation());
        });

        // Congigure triggers
//        driver.start.onFalling(robot.drive::resetGyro);
//        driver.back.onFalling(
//                robot.drive
//                        ::resetGyro); // Two buttons to reset gyro so the driver can't get confused

//        new Trigger(() -> driver.dpad.up.isPressed()).whileTrue(new InstantCommand());
    }

    public Vec2d getDriveTranslation() {
        double x = -driver.getLeftStick().y;
        double y = -driver.getLeftStick().x;

        return new Vec2d(x, y).deadband(DEADBAND);
    }

    public double getDriveRotation() {
        return deadband(driver.rightStickX.get());
    }

    /**
     * Pre-process inputs from joysticks
     *
     * @param val Joystick axis input
     * @return Deadbanded output
     */
    private double deadband(double val) {
        return MathUtil.deadband(val, DEADBAND);
    }
}
