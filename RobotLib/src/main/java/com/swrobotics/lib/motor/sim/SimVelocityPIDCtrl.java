package com.swrobotics.lib.motor.sim;

import com.swrobotics.lib.encoder.SimEncoder;
import com.swrobotics.mathlib.Angle;

public final class SimVelocityPIDCtrl implements SimControlMode<Angle> {
    private final SimPIDController pid;
    private final SimEncoder encoder;
    private final double sensorUnitsPerRPS;

    public SimVelocityPIDCtrl(SimPIDController pid, SimEncoder encoder, double sensorUnitsPerRPS) {
        this.pid = pid;
        this.encoder = encoder;
        this.sensorUnitsPerRPS = sensorUnitsPerRPS;
    }

    @Override
    public void begin() {
        pid.reset();
    }

    @Override
    public double calc(Angle demand) {
        return pid.calc(
                encoder.getVelocity().ccw().rot() * sensorUnitsPerRPS,
                demand.ccw().rot() * sensorUnitsPerRPS);
    }
}
