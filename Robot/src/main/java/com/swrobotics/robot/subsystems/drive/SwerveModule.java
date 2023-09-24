package com.swrobotics.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

public class SwerveModule { // Intentionally not a subsystem
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final int index;

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    public void periodic() {
        io.updateInputa(inputs);
        Logger.getInstance().processInputs("Drive/Module" + index, inputs);
    }
}
