package com.swrobotics.robot.subsystems.speaker.aim;

public interface AimCalculator {
    // Flywheel velocity is in rotations/second
    // Pivot angle is in radians
    final record Aim(double flywheelVelocity, double pivotAngle) {}

    // Distance is in meters
    // May return null if shot is impossible
    Aim calculateAim(double distanceToSpeaker);
}
