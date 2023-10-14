package com.swrobotics.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] modules;

    public Drive(GyroIO gyroIO, SwerveModuleManager manager) {
        System.out.println("[Init] Creating Drive");
        this.gyroIO = gyroIO;
        modules = manager.getModules();
    }

    public void resetGyro() {}

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);

        /* Run module periodic from here
         * allows modules to be turned off with drive being turned off
         */
        for (var module : modules) {
            module.periodic();
        }
    }
}
