package com.swrobotics.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class ModuleIOFalcon implements SwerveModuleIO {

    public ModuleIOFalcon(int index) {
        System.out.println("[Init] Creating Falcon Module " + index);
    }

    public static class FalconModuleConstants {
        public int longCANTimeoutMs = 100;

        // IDs
        public int driveMotorID = 0;
        public int turnMotorID = 0;
        public int encoderID = 0;
        public double offset = 0.0;

        // Angle config
        public boolean angleEnableCurrentLimit = true;
        public double angleContinuousCurrentLimit = 25;
        public double anglePeakCurrentLimit = 40;
        public double anglePeakCurrentDuration = 0.1;

        public TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    }
}
