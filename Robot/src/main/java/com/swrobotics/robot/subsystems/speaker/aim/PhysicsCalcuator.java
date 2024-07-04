package com.swrobotics.robot.subsystems.speaker.aim;

import com.swrobotics.robot.subsystems.speaker.ShooterSubsystem2.ShootingState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PhysicsCalcuator {
    private static final double g = 9.8; // ms^2
    public Rotation2d calculate(Translation2d point, ShootingState state) {
        double v = state.currentFlywheelVelocityMPS();
        double x = point.getX();
        double y = point.getY();

        double theta = Math.atan2(v*v - Math.sqrt(Math.pow(v,4) - g * (g * x*x + 2 * y * v*v))
                                    , g * x);
        return Rotation2d.fromRadians(theta);
    }
}
