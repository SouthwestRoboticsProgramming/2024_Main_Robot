package com.swrobotics.blockauto.tool.nt;

import edu.wpi.first.networktables.NetworkTableInstance;

public interface NTInstanceListener {
    void onNTInit(NetworkTableInstance inst);

    void onNTClose();
}
