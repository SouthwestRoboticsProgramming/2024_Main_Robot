package com.swrobotics.robot.subsystems.drive;

import com.swrobotics.mathlib.Angle;
import com.swrobotics.robot.config.CANAllocation;
import com.swrobotics.robot.config.Settings;
import com.swrobotics.robot.config.CANAllocation.SwerveIDs;
import com.swrobotics.robot.config.Settings.Mode;
import com.swrobotics.robot.subsystems.drive.ModuleIOFalcon.FalconModuleConstants;

/** Manages hot-swappable modules and module constants */
public class SwerveModuleManager {
    // private static final SwerveModuleInfo[] MODULES = new SwerveModuleInfo[] {
    //     new SwerveModuleInfo("Front Left", CANAllocation.SWERVE_FL, Angle.ZERO), // FIXME: Do angle offset
    //     new SwerveModuleInfo("Front Right", CANAllocation.SWERVE_FR, Angle.ZERO), // FIXME: Do angle offset
    //     new SwerveModuleInfo("Back Left", CANAllocation.SWERVE_BL, Angle.ZERO), // FIXME: Do angle offset
    //     new SwerveModuleInfo("Back Right", CANAllocation.SWERVE_BR, Angle.ZERO), // FIXME: Do angle offset
    // };

    private static final String[] MODULE_NAMES = new String[] {"Front Left", "Front Right", "Back Left", "Back Right"};

    public SwerveModule[] getModules() {
        
        // if (Settings.getMode() != Mode.REPLAY){
        //     switch (Settings.robot) {
        //         case COMPETITION:
        //             return new SwerveModule[4];        
        //         case SIMULATION:
        //             return new SwerveModule[4];
        //     }
        // }

        // Empty IOs for replay mode. They act as a false module
        return createModulesFromIO(new SwerveModuleIO[] {
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {}
        }, MODULE_NAMES);
    }

    private static SwerveModule[] createModulesFromIO(SwerveModuleIO[] ios, String[] names) {
        SwerveModule[] modules = new SwerveModule[ios.length];
        for (int i = 0; i < ios.length; i++) {
            modules[i] = new SwerveModule(ios[i], names[i]);
        }

        return modules;
    }

    public static class SwerveModuleInfo {
        public final String name;
        public final SwerveIDs ids;
        public final Angle angleOffset;
    
        public SwerveModuleInfo(String name, SwerveIDs ids, Angle angleOffset) {
            this.name = name;
            this.ids = ids;
            this.angleOffset = angleOffset;
        }
    }
}
