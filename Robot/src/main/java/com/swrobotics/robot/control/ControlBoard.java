package com.swrobotics.robot.control;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.swrobotics.lib.input.XboxController;
import com.swrobotics.robot.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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
        // Passing DEADBAND here means we don't have to deadband anywhere else
        driver = new XboxController(0, DEADBAND);
        operator = new XboxController(1, DEADBAND);

        driver.x.onRising(AutoBuilder.pathfindToPose(new Pose2d(10, 4, new Rotation2d(0)), new PathConstraints(4, 8, 10, 40)));

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
