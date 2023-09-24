package com.swrobotics.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] modules;
    
    public Drive(GyroIO gyroIO, SwerveModuleManager manager) {
        System.out.println("[Init] Creating Drive");
        this.gyroIO = gyroIO;
        modules = manager.getModules();
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
    }
}
