package com.swrobotics.lib.motor.sim;

public final class SimPercentOutCtrl implements SimControlMode<Double> {
    @Override
    public double calc(Double demand) {
        return demand;
    }
}
