package com.swrobotics.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import com.swrobotics.lib.net.NTEntry;

import edu.wpi.first.math.geometry.Rotation2d;
import com.swrobotics.robot.config.CANAllocation;
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
        public Info(CANAllocation.SwerveIDs ids, double x, double y, NTEntry<Double> offset, String name) {
            this(ids.drive, ids.turn, ids.encoder,
                    new Translation2d(x, y),
                    offset, name);
        }
    }

    private final Info info;
    private SwerveModuleIO hardwareIO;
    private SwerveModuleIO.SwerveModuleIOInputs inputs;

    public SwerveModule(SwerveModuleIO hardwareIO, Info info) {
        this.hardwareIO = hardwareIO;
        this.info = info;
        inputs = new SwerveModuleIO.SwerveModuleIOInputs();
    }
    
    public void setTargetState(SwerveModuleState targetState) {
        // Optimized to avoid spinning the turn motors too much
        targetState = optimizeSwerveModuleState(targetState);

        // Modify the target
        Rotation2d targetRotation = targetState.angle;

        Rotation2d newAngle = Rotation2d.fromDegrees(placeInAppropriate0To360Scope(getCurrentState().angle.getDegrees(), targetRotation.getDegrees()));

        targetState = new SwerveModuleState(targetState.speedMetersPerSecond, newAngle);
        
        inputs.targetDriveVelocityMetersPerSec = targetState.speedMetersPerSecond;
        inputs.targetSteerPositionRad = targetState.angle.getRadians();
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(inputs.driveVelocityMetersPerSec, new Rotation2d(inputs.steerPositionRad));
    }

    public SwerveModuleState getTargetState() {
        return new SwerveModuleState(inputs.targetDriveVelocityMetersPerSec, new Rotation2d(inputs.targetSteerPositionRad));
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(inputs.drivePositionMeters, new Rotation2d(inputs.steerPositionRad));
    }
    public double getMaxVelocity() {
        return hardwareIO.getMaxVelocity();
    }

    public void update() {
        hardwareIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/" + info.name() + " Module", inputs);
    }
    

    private SwerveModuleState optimizeSwerveModuleState(SwerveModuleState targetState) {
        Rotation2d targetAngle = targetState.angle;
        Rotation2d currentAngle = getCurrentState().angle;

        double targetVelocity = targetState.speedMetersPerSecond;
        if (shouldReverse(targetAngle, currentAngle)) {
            targetVelocity = -targetVelocity;
            targetAngle.plus(Rotation2d.fromDegrees(180));
        }
        
        return new SwerveModuleState(targetVelocity, targetAngle);
    }

    private static boolean shouldReverse(Rotation2d goalAngle, Rotation2d currentAngle) {
        double angleDifference = Math.abs(distance(goalAngle, currentAngle));
        double reverseAngleDifference = Math.abs(distance(goalAngle, currentAngle.rotateBy(Rotation2d.fromDegrees(180.0))));
        return reverseAngleDifference < angleDifference;
    }

    private static double distance(Rotation2d main, Rotation2d other) {
        return main.unaryMinus().rotateBy(other).getRadians();
    }

    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }
    
}


    // TODO
    // /** CTRE-Specific signals directly from the motor controllers and encoders */
    // public BaseStatusSignal[] getSignals() {
    //     return new BaseStatusSignal[0];
    // }
