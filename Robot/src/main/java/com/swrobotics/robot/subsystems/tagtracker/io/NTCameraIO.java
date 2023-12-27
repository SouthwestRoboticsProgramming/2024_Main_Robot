package com.swrobotics.robot.subsystems.tagtracker.io;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;

public final class NTCameraIO implements TagTrackerCameraIO {
    private final DoubleArraySubscriber sub;

    public NTCameraIO(NetworkTable table) {
        sub = table.getDoubleArrayTopic("poses").subscribe(
                new double[] {0},
                PubSubOption.sendAll(true),
                PubSubOption.keepDuplicates(true));
    }

    @Override
    public void updateInputs(Inputs inputs) {
        TimestampedDoubleArray[] data = sub.readQueue();
        inputs.timestamps = new long[data.length];
        inputs.framePackedData = new double[data.length][];

        for (int i = 0; i < data.length; i++) {
            inputs.timestamps[i] = data[i].timestamp;
            inputs.framePackedData[i] = data[i].value;
        }
    }
}
