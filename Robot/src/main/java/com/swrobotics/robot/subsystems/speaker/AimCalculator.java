package com.swrobotics.robot.subsystems.speaker;

import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;

import static com.swrobotics.mathlib.MathUtil.G_ACCEL;

public final class AimCalculator {
    public static final double velocity = 9.97557;

    private static final double gSquaredQuarter = G_ACCEL * G_ACCEL / 4;

    private static final double speakerBottom = 1.98;
    private static final double speakerTop = 2.11;
    private static final double speakerAngle = Math.toRadians(14);
    private static final double speakerHeight = speakerTop - speakerBottom;
    private static final double speakerWidth = speakerHeight / Math.tan(speakerAngle);

    private static final double releaseHeight = 0.25; // TODO: Measure in CAD
    private static final double targetY = (speakerBottom + speakerTop) / 2 - releaseHeight;
    private static final double targetXOffset = -speakerWidth / 2;

    // Slight correction for fun
//    private static final double angleOffset = 0; // TODO: NT
    private static final NTEntry<Double> angleOffset = new NTDouble("Shooter/Aim Offset", 0).setPersistent();

    // Flywheel velocity is in meters/second
    // Pivot angle is in radians
    public static final record Aim(double flywheelVelocity, double pivotAngle) {}

    // TODO: Account for robot velocity?
    public static Aim calculateAim(double distToSpeaker) {
        // TODO: Maybe slow down when close?
        double v = velocity;

        double targetX = distToSpeaker + targetXOffset;

        double a = gSquaredQuarter;
        double b = G_ACCEL * targetY - v * v;
        double c = targetX * targetX + targetY * targetY;
        double timeSquared = (-b - Math.sqrt(b * b - 4 * a * c)) / (2 * a);
        double time = Math.sqrt(timeSquared);

        double velocityX = targetX / time; // TODO: Account for robot velocity towards speaker
        double angle = Math.acos(velocityX / v);

        // TODO: Return null if it's impossible
//        return new Aim(v, angle + Math.toRadians(angleOffset.get()));
        logDistance.set(distToSpeaker);
        return new Aim(tuneVelocity.get(), Math.toRadians(tunePivotAngle.get()));
    }

    static NTDouble tunePivotAngle = new NTDouble("Shooter/Tuning/Pivot Angle (deg)", 30);
    static NTDouble tuneVelocity = new NTDouble("Shooter/Tuning/Flywheel Velocity (rps)", 3000);
    static NTDouble logDistance = new NTDouble("Shooter/Tuning/Est Distance (m)", 0);
}
