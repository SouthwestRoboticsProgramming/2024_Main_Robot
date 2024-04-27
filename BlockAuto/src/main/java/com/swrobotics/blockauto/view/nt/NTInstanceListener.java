package com.swrobotics.blockauto.view.nt;

import edu.wpi.first.networktables.NetworkTableInstance;

public interface NTInstanceListener {
    void onNTInit(NetworkTableInstance inst);

    void onNTClose();
}
