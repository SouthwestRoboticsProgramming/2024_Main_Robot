package com.swrobotics.robot.subsystems.speaker.aim;

import com.swrobotics.mathlib.MathUtil;

import java.util.TreeMap;

public final class TableAimCalculator implements AimCalculator {
    private final TreeMap<Double, Double> flywheelVelocityMap = new TreeMap<>();
    private final TreeMap<Double, Double> pivotAngleMap = new TreeMap<>();

    public TableAimCalculator() {
        // TODO: More datas (this data may also be incorrect)
        addCalibrationSample(1.1429, 63, 35);
        addCalibrationSample(1.924, 54, 39);
        addCalibrationSample(2.765, 41, 45);
        addCalibrationSample(3.635, 34, 70); // vroom
    }

    private void addCalibrationSample(double distanceM, double angleDeg, double velocityRPS) {
        flywheelVelocityMap.put(distanceM, velocityRPS);
        pivotAngleMap.put(distanceM, Math.toRadians(angleDeg));
    }

    private double sample(TreeMap<Double, Double> map, double distanceToSpeaker) {
        if (map.isEmpty())
            throw new IllegalStateException("The aiming table has no data!");

        Double distCloser = map.floorKey(distanceToSpeaker);
        Double distFarther = map.ceilingKey(distanceToSpeaker);

        // Clamp to min or max known sample if out of range
        if (distCloser == null)
            distCloser = distFarther;
        if (distFarther == null)
            distFarther = distCloser;

        // Prevent divide by 0 in MathUtil.percent() below
        if (distCloser.equals(distFarther))
            return map.get(distCloser);

        // Interpolate between neighboring samples
        double percent = MathUtil.percent(distanceToSpeaker, distCloser, distFarther);
        double valCloser = map.get(distCloser);
        double valFarther = map.get(distFarther);
        return MathUtil.lerp(valCloser, valFarther, percent);
    }

    @Override
    public Aim calculateAim(double distanceToSpeaker) {
        return new Aim(
                sample(flywheelVelocityMap, distanceToSpeaker),
                sample(pivotAngleMap, distanceToSpeaker)
        );
    }
}
