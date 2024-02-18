package com.swrobotics.lib.net;

import edu.wpi.first.math.controller.PIDController;

public final class NTUtil {
    public static PIDController tunablePID(
            NTEntry<Double> kP, NTEntry<Double> kD) {
        PIDController pid = new PIDController(kP.get(), 0, kD.get());
        kP.onChange(pid::setP);
        kD.onChange(pid::setD);
        return pid;
    }
}
