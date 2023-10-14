package com.swrobotics.shufflelog.tool.data.nt;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Equivalent to GenericSubscriber, but automatically recreates when NT disconnects and reconnects.
 */
public final class NTSubscriber {
    private final String path;
    private GenericSubscriber sub;

    public NTSubscriber(String path) {
        this.path = path;
    }

    public double[] getDoubleArray(double[] defaultVal) {
        if (sub == null) return defaultVal;
        return sub.getDoubleArray(defaultVal);
    }

    void init(NetworkTableInstance instance) {
        sub = instance.getTopic(path).genericSubscribe();
    }

    void close() {
        sub = null;
    }
}
