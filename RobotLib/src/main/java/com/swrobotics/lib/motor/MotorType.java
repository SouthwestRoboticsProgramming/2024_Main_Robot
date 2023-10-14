package com.swrobotics.lib.motor;

import com.swrobotics.mathlib.Angle;
import com.swrobotics.mathlib.CCWAngle;

import edu.wpi.first.math.util.Units;

/** Class to store the properties of different types of motors (not motor controllers!). */
public final class MotorType {
    public static final MotorType FALCON_500 =
            new MotorType(
                    CCWAngle.rad(Units.rotationsPerMinuteToRadiansPerSecond(6380)), 2048, 2048);
    public static final MotorType NEO =
            new MotorType(CCWAngle.rad(Units.rotationsPerMinuteToRadiansPerSecond(5676)), 1, 1);
    public static final MotorType NEO_550 =
            new MotorType(CCWAngle.rad(Units.rotationsPerMinuteToRadiansPerSecond(11000)), 1, 1);

    // TODO: More accurate physical representation
    public final Angle freeSpeed;

    // TODO: Remove after refactoring to use Phoenix v6
    public final double sensorUnitsPerRot;
    public final double sensorUnitsPerRPS;

    public MotorType(Angle freeSpeed, double sensorUnitsPerRot, double sensorUnitsPerRPS) {
        this.freeSpeed = freeSpeed;
        this.sensorUnitsPerRot = sensorUnitsPerRot;
        this.sensorUnitsPerRPS = sensorUnitsPerRPS;
    }
}
