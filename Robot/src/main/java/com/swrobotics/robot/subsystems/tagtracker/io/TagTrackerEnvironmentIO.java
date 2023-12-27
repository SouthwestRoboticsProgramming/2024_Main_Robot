package com.swrobotics.robot.subsystems.tagtracker.io;

import com.swrobotics.robot.logging.AutoLoggedInputs;

public interface TagTrackerEnvironmentIO {
    void updateInputs(Inputs inputs);

    final class Inputs extends AutoLoggedInputs {
        public boolean dataChanged = false;
        public double[] packedData = new double[0];
    }
}
