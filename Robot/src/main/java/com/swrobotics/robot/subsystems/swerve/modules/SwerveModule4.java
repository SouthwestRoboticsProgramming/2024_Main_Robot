package com.swrobotics.robot.subsystems.swerve.modules;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.config.CANAllocation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule4 {

    public record Info(
            int driveId, int steerId, int encoderId,
            Translation2d position,
            NTEntry<Double> offset,
            String canbus,
            String name) {
        public Info(CANAllocation.SwerveIDs ids, double x, double y, NTEntry<Double> offset, String canbus, String name) {
            this(ids.drive, ids.turn, ids.encoder,
                    new Translation2d(x, y),
                    offset, canbus, name);
        }
    }

    private final String name;

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder encoder;

    private SwerveModuleState targetState = new SwerveModuleState();
    private SwerveModulePosition currentPosition = new SwerveModulePosition();

    public SwerveModule4(SwerveModuleConstants constants, String canbus, String name) {
        this.name = name;

        driveMotor = new TalonFX(constants.DriveMotorId, canbus);
        steerMotor = new TalonFX(constants.SteerMotorId, canbus);
        encoder = new CANcoder(constants.CANcoderId, canbus);

        // Configure the drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Default to brake mode

        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        StatusCode response = driveMotor.getConfigurator().apply(driveConfig);
        if (!response.isOK()) {
            System.out.println("Drive motor on " + name + " module (ID " + constants.DriveMotorId + ") failed config with error " + response.toString());
        }

        // Configure steer motor
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        // Modify configuration to use remote CANcoder fused
        steerConfig.Feedback.FeedbackRemoteSensorID = constants.CANcoderId;
        switch (constants.FeedbackSource) {
            case RemoteCANcoder:
                steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                break;
            case FusedCANcoder:
                steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                break;
            case SyncCANcoder:
                steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
                break;
        }

        steerConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;

        steerConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
        steerConfig.MotionMagic.MotionMagicAcceleration = steerConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        steerConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
        steerConfig.MotionMagic.MotionMagicExpo_kA = 0.1;

        steerConfig.ClosedLoopGeneral.ContinuousWrap = true; // Enable continuous wrap for swerve modules

        steerConfig.MotorOutput.Inverted = constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        response = steerMotor.getConfigurator().apply(steerConfig);
        if (!response.isOK()) {
            System.out.println("Steer motor on " + name + " module (ID " + constants.SteerMotorId + ") failed config with error " + response.toString());
        }

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
        response = encoder.getConfigurator().apply(encoderConfig);
        if (!response.isOK()) {
            System.out.println(
                    "CANcoder on " + name + " modules (ID " + constants.DriveMotorId + ") failed config with error " + response.toString());
        }
    }

    public void setTargetState(SwerveModuleState state, DriveRequestType driveRequestType) {
        setTargetState(state, driveRequestType, SteerRequestType.MotionMagic);
    }

    public void setTargetState(SwerveModuleState state, DriveRequestType driveRequestType, SteerRequestType steerRequestType) {
        // TODO: Do stuff
        targetState = state;
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public SwerveModuleState getCurrentState() {
        // TODO: Do
        return targetState;
    }

    public SwerveModulePosition getCurrentPosition() {
        currentPosition.angle = targetState.angle;
        currentPosition.distanceMeters += targetState.speedMetersPerSecond * 0.02;
        return currentPosition;
    }

    /**
     * Sets how the drive motor should behave when not being demanded to an output
     * @param neutralMode The new mode that the drive motor should assume
     */
    public void setDriveBrakeMode(NeutralModeValue neutralMode) {
        driveMotor.setNeutralMode(neutralMode);
    }
}
