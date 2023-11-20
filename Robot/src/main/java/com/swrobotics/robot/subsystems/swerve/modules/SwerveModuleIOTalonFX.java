package com.swrobotics.robot.subsystems.swerve.modules;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.swrobotics.robot.subsystems.swerve.modules.SwerveModule.Info;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    
    private final CANcoder encoder;

    private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut voltageControl = new VoltageOut(0.0, true, false);
    private final PositionVoltage turnPositionControl = new PositionVoltage(0);

    public SwerveModuleIOTalonFX(Info info, String canbus) {
        driveMotor = new TalonFX(info.driveId(), canbus);
        turnMotor = new TalonFX(info.turnId(), canbus);
        encoder = new CANcoder(info.encoderId(), canbus);

        // Configure the CANCoder
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = info.offset().get();
        encoder.getConfigurator().apply(encoderConfig);

        // Configure the drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Slot0 = new Slot0Configs();
        driveMotor.getConfigurator().apply(driveConfig);

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.TorqueCurrent = new TorqueCurrentConfigs();
        turnConfig.Slot0 = new Slot0Configs();
        turnConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnConfig.Feedback.RotorToSensorRatio = 0;
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs swerveModuleIOInputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public double getMaxVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMaxVelocity'");
    }
    
}
