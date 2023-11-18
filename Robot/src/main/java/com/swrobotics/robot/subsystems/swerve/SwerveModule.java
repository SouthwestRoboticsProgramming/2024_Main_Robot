package com.swrobotics.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
/**
 * Interfaces between the drive controller and the actual module
 * - Also does the heavy lifting when it comes to math
 */
public class SwerveModule {
    record Info(
        int driveId, int turnId, int encoderId,
        Translation2d position,
        NTEntry<Double> offset,
        String name) {
        public Info(int driveId, int turnId, int encoderId, double x, double y, String name) {
            this(driveId, turnId, encoderId,
                    new Translation2d(x, y),
                    new NTDouble("Swerve/Modules/" + name + " Offset", 0).setPersistent(),
                    name);
        }
    }

    private final Info info;
    private SwerveModuleIO hardwareIO;
    private SwerveModuleIOInputsAutoLogged inputs;

    public SwerveModule(SwerveModuleIO hardwareIO, Info info) {
        this.hardwareIO = hardwareIO;
        this.info = info;
        inputs = new SwerveModuleIOInputsAutoLogged();
    }
    
    public void setTargetState(SwerveModuleState targetState) {
        // TODO: Optimize
        inputs.targetDriveVelocityMetersPerSec = targetState.speedMetersPerSecond;
        inputs.targetSteerPositionRad = targetState.angle.getRadians();
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(inputs.driveVelocityMetersPerSec, new Rotation2d(inputs.steerPositionRad));
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(inputs.drivePositionMeters, new Rotation2d(inputs.steerPositionRad));
    }

    public void update() {
        hardwareIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/" + info.name() + " Module", inputs);
    }

    // TODO
    // /** CTRE-Specific signals directly from the motor controllers and encoders */
    // public BaseStatusSignal[] getSignals() {
    //     return new BaseStatusSignal[0];
    // }
}
