package com.swrobotics.shufflelog.tool.data.nt;

import edu.wpi.first.networktables.NetworkTableInstance;

public interface NTInstanceListener {
    void onNTInit(NetworkTableInstance inst);

    void onNTClose();
}
