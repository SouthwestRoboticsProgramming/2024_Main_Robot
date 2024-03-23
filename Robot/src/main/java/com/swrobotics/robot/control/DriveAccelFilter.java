package com.swrobotics.robot.control;

import com.swrobotics.lib.net.NTDouble;
import edu.wpi.first.wpilibj.Timer;

public final class DriveAccelFilter {
    private final double maxAccel;
    private double currentVelocity;

    private double prevTimestamp = Double.NaN;

    /**
     * @param maxAccel maximum acceleration per second
     */
    public DriveAccelFilter(double maxAccel) {
        if (maxAccel <= 0)
            throw new IllegalArgumentException("maxAccel must be positive");
        this.maxAccel = maxAccel;

        currentVelocity = 0;
    }

    public double calculate(double targetVelocity) {
        // Allow instant deceleration
        if (Math.abs(currentVelocity) > Math.abs(targetVelocity)) {
            currentVelocity = targetVelocity;
            return targetVelocity;
        }

        double time = Timer.getFPGATimestamp();
        double delta = 0;
        if (!Double.isNaN(prevTimestamp))
            delta = time - prevTimestamp;
        prevTimestamp = time;

        double error = targetVelocity - currentVelocity;
        double toChange = Math.min(maxAccel * delta, Math.abs(error));

        currentVelocity += Math.copySign(toChange, error);

        return currentVelocity;
    }

    public void reset(double currentVelocity) {
        this.currentVelocity = currentVelocity;
    }
}
