package com.swrobotics.lib.motor.sim;

import com.swrobotics.lib.encoder.SimEncoder;
import com.swrobotics.mathlib.Angle;

public final class SimPositionPIDCtrl implements SimControlMode<Angle> {
    private final SimPIDController pid;
    private final SimEncoder encoder;
    private final double sensorUnitsPerRot;

    public SimPositionPIDCtrl(SimPIDController pid, SimEncoder encoder, double sensorUnitsPerRot) {
        this.pid = pid;
        this.encoder = encoder;
        this.sensorUnitsPerRot = sensorUnitsPerRot;
    }

    @Override
    public void begin() {
        pid.reset();
    }

    @Override
    public double calc(Angle demand) {
        return pid.calc(
                encoder.getAngle().ccw().rot() * sensorUnitsPerRot,
                demand.ccw().rot() * sensorUnitsPerRot);
    }
}
