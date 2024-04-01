package com.swrobotics.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

public class SwerveConstants {
    // Both sets of gains need to be tuned to your individual robot.
    // TODO: tune them to the robot

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.05)
        .withKS(0).withKV(0).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    // TODO: tune to the robot
    private static final double slipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double speedAt12VoltsMps = SwerveDrive.MAX_LINEAR_SPEED;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double coupleRatio = 3.5;

    private static final double driveGearRatio = (50.0/16) * (16.0/28) * (45.0/15);
    private static final double steerGearRatio = 150.0 / 7;
    private static final double wheelRadiusInches = 1.9 * 0.981162 * 1.022136 * 0.991 * 1.004198; // Estimated at first, then fudge-factored to make odom match record

    private static final boolean steerMotorReversed = true;


    // These are only used for simulation
    private static final double steerInertia = 0.00001;
    private static final double driveInertia = 0.001;

    public static final SwerveModuleConstantsFactory SWERVE_MODULE_BUILDER = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(driveGearRatio)
            .withSteerMotorGearRatio(steerGearRatio)
            .withWheelRadius(wheelRadiusInches)
            .withSlipCurrent(slipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(speedAt12VoltsMps)
            .withSteerInertia(steerInertia)
            .withDriveInertia(driveInertia)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(coupleRatio)
            .withSteerMotorInverted(steerMotorReversed);
}
