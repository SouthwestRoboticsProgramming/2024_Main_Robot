package com.swrobotics.lib.motor.sim;

public interface SimControlMode<D> {
    default void begin() {}

    double calc(D demand);
}
