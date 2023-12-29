package com.swrobotics.robot.control;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.swrobotics.lib.input.XboxController;
import com.swrobotics.robot.RobotContainer;

import com.swrobotics.robot.subsystems.swerve.pathfinding.CircleShape;
import com.swrobotics.robot.subsystems.swerve.pathfinding.RectangleShape;
import com.swrobotics.robot.subsystems.swerve.pathfinding.Shape;
import com.swrobotics.robot.subsystems.swerve.pathfinding.ThetaStarPathfinder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

        // Passing DEADBAND here means we don't have to deadband anywhere else
        driver = new XboxController(0, DEADBAND);
        operator = new XboxController(1, DEADBAND);

        // Congigure triggers
       driver.start.onFalling(() -> robot.drive.setRotation(new Rotation2d()));
       driver.back.onFalling(() -> robot.drive.setRotation(new Rotation2d())); // Two buttons to reset gyro so the driver can't get confused
    }

    public Translation2d getDriveTranslation() {
        Translation2d leftStick = driver.getLeftStick();
        double x = -leftStick.getY();
        double y = -leftStick.getX();
        return new Translation2d(x, y);
    }

    public double getDriveRotation() {
        return -driver.rightStickX.get();
    }
}
