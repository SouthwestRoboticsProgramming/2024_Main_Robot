package com.swrobotics.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

public class SwerveModule { // Intentionally not a subsystem
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final String name;

    public SwerveModule(SwerveModuleIO io, String name) {
        this.io = io;
        this.name = name;
    }

    public void periodic() {
        io.updateInputa(inputs);
        Logger.getInstance().processInputs("Drive/" + name + " Module", inputs);
    }
}
