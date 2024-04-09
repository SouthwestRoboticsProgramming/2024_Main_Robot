package com.swrobotics.robot.subsystems.speaker.aim;

import java.util.TreeMap;

import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.NTData;

import edu.wpi.first.math.util.Units;

public final class TableAimCalculator implements AimCalculator {
    public static final TableAimCalculator INSTANCE = new TableAimCalculator();

    private final TreeMap<Double, Double> flywheelVelocityMap = new TreeMap<>();
    private final TreeMap<Double, Double> pivotAngleMap = new TreeMap<>();

   private static final NTDouble logDistance = new NTDouble("Shooter/Manual Aim/Est Distance (m)", 0);

    public TableAimCalculator() {
        // Calibration 1 (MURA)
//        double off = 5;
//        addCalibratio?nSample(1.1429, 63 - off, 35);

//        addCalibrationSample(1.22, 60, 39);
//        addCalibrationSample(2.1, 46, 39);
//        addCalibrationSample(3.0, 34, 60);
//        addCalibrationSample(3.8, 29, 70);

        double fieldWrongness = Units.inchesToMeters(1 + 5.0/8); 

        // MURA 3-16, 3|1 wheel shooter configuration
        addCalibrationSample(fieldWrongness + 1.224203 - .2, 58, 55);
        addCalibrationSample(fieldWrongness + 1.657 - .2, 52, 55);
        addCalibrationSample(fieldWrongness + 2.239 - .2, 44, 55);
        addCalibrationSample(fieldWrongness + 2.875 - .2, 36, 55);
        addCalibrationSample(fieldWrongness + 2.993, 34, 63);
        addCalibrationSample(fieldWrongness + 3.506, 30, 63);
        addCalibrationSample(fieldWrongness + 3.903, 28, 67);

        // MURA 3-23, 3|1 wheel shooter
        addCalibrationSample(fieldWrongness + 4.34, 28 + 1, 67);
        addCalibrationSample(fieldWrongness + 4.67, 27 + 1, 67);
        addCalibrationSample(fieldWrongness + 5.06, 25 + 1, 67);
        addCalibrationSample(fieldWrongness + 5.58, 24 + 1, 70);

        addCalibrationSample(fieldWrongness + 6.6, 22.5, 82);

//        addCalibrationSample(3.37, 29, 60);

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
        if (distFarther == null) {
            double angle = Math.toRadians(MathUtil.clamp(Math.pow(0.578366, distanceToSpeaker - 7.90303) + 20.21, 22, 40));
            double velocity = MathUtil.clamp(distanceToSpeaker * 3.94923 + 48.5462, 10, 100);
            
            if (map == flywheelVelocityMap) { return velocity;}
            if (map == pivotAngleMap) { return angle; }
            distFarther = distCloser;
        }


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
                sample(pivotAngleMap, distanceToSpeaker),
                distanceToSpeaker
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
