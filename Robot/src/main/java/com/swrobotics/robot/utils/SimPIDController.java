package com.swrobotics.robot.utils;

import com.swrobotics.mathlib.MathUtil;

public final class SimPIDController {
    private final double calcInterval;
    private final double outputScale;

    public SimPIDController(double calcInterval, double outputScale) {
        this.calcInterval = calcInterval;
        this.outputScale = outputScale;
    }

    // PIDF constant units:
    // kP: output per error
    // kI: (output per error) per period
    // kD: output per (error per period)
    // kF: output per setpoint
    private double kP, kI, kD, kF;
    private double integralAcc, prevError;

    // Setpoint and measure are in native sensor units
    public double calc(double measure, double setpoint) {
        double error = setpoint - measure;
        double p = error * kP;

        // error = inc per 0.02s, equivalent to (inc per period) * (period per 0.02s)
        integralAcc += error * kI;

        // (error - prevError) = delta per 0.02s
        // (error - prevError) / 0.02 = delta per 1s
        // (error - prevError) / 0.02 * caps.pidCalcInterval = delta per interval
        double d = (error - prevError) / 0.02 * calcInterval * kD;
        prevError = error;

        double f = setpoint * kF;

        return MathUtil.clamp((p + integralAcc + d + f) / outputScale, -1, 1);
    }

    public void reset() {
        integralAcc = 0;
        prevError = 0;
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public void setI(double kI) {
        this.kI = kI;
    }

    public void setD(double kD) {
        this.kD = kD;
    }

    public void setF(double kF) {
        this.kF = kF;
    }
}

