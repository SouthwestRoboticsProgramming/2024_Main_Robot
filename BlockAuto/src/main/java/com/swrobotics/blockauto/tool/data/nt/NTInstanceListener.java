package com.swrobotics.blockauto.tool.data.nt;

import edu.wpi.first.networktables.NetworkTableInstance;

public interface NTInstanceListener {
    void onNTInit(NetworkTableInstance inst);

    void onNTClose();
}
