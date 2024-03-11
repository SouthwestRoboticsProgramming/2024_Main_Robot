package com.swrobotics.robot.subsystems.speaker.aim;

import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.NTData;

import java.util.TreeMap;

public final class TableAimCalculator implements AimCalculator {
    public static final TableAimCalculator INSTANCE = new TableAimCalculator();

    private final TreeMap<Double, Double> flywheelVelocityMap = new TreeMap<>();
    private final TreeMap<Double, Double> pivotAngleMap = new TreeMap<>();

   private static final NTDouble logDistance = new NTDouble("Shooter/Manual Aim/Est Distance (m)", 0);

    public TableAimCalculator() {
        // Calibration 1 (MURA)
        double off = 5;
//        addCalibratio?nSample(1.1429, 63 - off, 35);

        addCalibrationSample(1.22, 60, 39);
        addCalibrationSample(2.1, 46, 39);
        addCalibrationSample(3.0, 34, 60);
        addCalibrationSample(3.8, 29, 70);

//        addCalibrationSample(1.924, 54 - off, 39);
//        addCalibrationSample(2.765, 41 - off, 45);
//        addCalibrationSample(3.635, 34 - off, 70); // vroom

        // Calibration 2 (shop)
//        addCalibrationSample(1.698725, 49.399451, 37.933578);
//        addCalibrationSample(2.340979, 38.664862, 44.923910);
//        addCalibrationSample(3.295272, 30.383333, 68.488096);

        // Calibration 3 (MURA)
//        addCalibrationSample(3.549, 31, 74);
//        addCalibrationSample(4.917, 24, 80);
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

//        System.out.printf("Sample (%s): close: (%.3f -> %.3f) far: (%.3f -> %.3f) pct %.3f\n", map == flywheelVelocityMap ? "V" : "A", distCloser, valCloser, distFarther, valFarther, percent);

        return MathUtil.lerp(valCloser, valFarther, percent);
    }

    @Override
    public Aim calculateAim(double distanceToSpeaker) {
       logDistance.set(distanceToSpeaker);

       distanceToSpeaker *= NTData.SHOOTER_DISTANCE_SCALE.get();

        return new Aim(
                sample(flywheelVelocityMap, distanceToSpeaker),
                sample(pivotAngleMap, distanceToSpeaker)
        );
    }

    public double getSnapDistance(double currentDist, boolean wantMoveCloser) {
        Double distCloser = pivotAngleMap.floorKey(currentDist);
        Double distFarther = pivotAngleMap.ceilingKey(currentDist);

        // Clamp to min or max known sample if out of range
        if (distCloser == null)
            distCloser = distFarther;
        if (distFarther == null)
            distFarther = distCloser;

        return wantMoveCloser ? distCloser : distFarther;
    }

    public boolean isTooFar(double distance) {
        return distance > pivotAngleMap.lastKey();
    }
}
