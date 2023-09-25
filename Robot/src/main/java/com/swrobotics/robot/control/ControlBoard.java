package com.swrobotics.robot.control;

import com.swrobotics.lib.input.XboxController;

public class ControlBoard {
    private static final double DEADBAND = 0.15;

    private static ControlBoard instance = null;

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

    // Ensures that only one instance is created
    public static ControlBoard getInstance() {
        if (instance == null) {
            instance = new ControlBoard();
        }

        return instance;
    }

    public final XboxController driver;
    public final XboxController operator;

    private ControlBoard() {
        driver = new XboxController(0, DEADBAND);
        operator = new XboxController(1, DEADBAND);
    }
}
