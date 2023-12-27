package com.swrobotics.robot.subsystems.tagtracker.io;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;

public final class NTEnvironmentIO implements TagTrackerEnvironmentIO {
    private final DoubleArraySubscriber sub;
    private long lastChangeTimestamp;

    public NTEnvironmentIO(DoubleArrayTopic topic) {
        sub = topic.subscribe(new double[0]);
        lastChangeTimestamp = Long.MAX_VALUE;
    }

    @Override
    public void updateInputs(Inputs inputs) {
        long timestamp = sub.getLastChange();
        inputs.dataChanged = timestamp != lastChangeTimestamp;
        if (!inputs.dataChanged)
            return;
        lastChangeTimestamp = timestamp;

        inputs.packedData = sub.get();
    }
}
