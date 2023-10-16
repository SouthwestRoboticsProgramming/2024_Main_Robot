package com.swrobotics.robot.control;

import com.swrobotics.lib.input.XboxController;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.mathlib.Vec2d;
import com.swrobotics.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        driver = new XboxController(0, DEADBAND);
        operator = new XboxController(1, DEADBAND);

        // Congigure triggers
        driver.start.onFalling(robot.drive::resetGyro);
        driver.back.onFalling(
                robot.drive
                        ::resetGyro); // Two buttons to reset gyro so the driver can't get confused

        new Trigger(() -> driver.dpad.up.isPressed()).whileTrue(new InstantCommand());
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
