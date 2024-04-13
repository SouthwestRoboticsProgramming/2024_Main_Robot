package com.swrobotics.robot.subsystems.tagtracker.io;

import com.swrobotics.robot.subsystems.tagtracker.CameraCaptureProperties;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TagTrackerCameraIO {
    void updateInputs(Inputs inputs);

    void setCaptureProperties(CameraCaptureProperties props);

    // Not AutoLoggedInputs since double array array is not auto-loggable
    final class Inputs implements LoggableInputs {
        public long[] timestamps;
        public double[][] framePackedData;

        @Override
        public void toLog(LogTable table) {
            table.put("timestamps", timestamps);
            table.put("frameCount", framePackedData.length);
            for (int i = 0; i < framePackedData.length; i++) {
                double[] frameData = framePackedData[i];
                table.put("frames/" + i, frameData);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            timestamps = table.get("timestamps", new long[0]);
            int frameCount = table.get("frameCount", 0);
            framePackedData = new double[frameCount][];
            for (int i = 0; i < frameCount; i++) {
                // Include one 0 as default, to indicate 0 detections that frame
                framePackedData[i] = table.get("frames/" + i, new double[] {0});
            }
        }
    }
}
