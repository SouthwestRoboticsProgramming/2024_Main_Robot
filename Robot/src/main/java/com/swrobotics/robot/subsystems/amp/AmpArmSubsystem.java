package com.swrobotics.robot.subsystems.amp;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.utils.TalonFXWithSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class AmpArmSubsystem extends SubsystemBase {
    public enum Position {
        STOW,
        PICKUP,
        SCORE
        // TODO: Does trap need a unique position?
    }

    private static final double motorToArmRatio = 2; // FIXME: not 2:1

    private final CANcoder absoluteEncoder = new CANcoder(IOAllocation.CAN.AMP_ARM_CANCODER.id(), IOAllocation.CAN.AMP_ARM_CANCODER.bus());
    private final TalonFXWithSim motor = new TalonFXWithSim(
            IOAllocation.CAN.AMP_ARM_MOTOR.id(),
            IOAllocation.CAN.AMP_ARM_MOTOR.bus(),
            DCMotor.getFalcon500Foc(1),
            motorToArmRatio,
            0.01)
            .attachCanCoder(absoluteEncoder);

    private final StatusSignal<Double> motorPosition;
    private final StatusSignal<Double> encoderPosition;

    public AmpArmSubsystem() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kP = 0.05;
        motorConfig.Slot0.kI = 0;
        motorConfig.Slot0.kD = 0;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // FIXME
        motorConfig.Feedback.SensorToMechanismRatio = motorToArmRatio;
        motor.getConfigurator().apply(motorConfig);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = 0;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // FIXME: Should match motor direction
        absoluteEncoder.getConfigurator().apply(encoderConfig);

        motorPosition = motor.getPosition();
        encoderPosition = absoluteEncoder.getAbsolutePosition();

        motor.setPosition(getPositionWithCanCoder());
    }

    public void setPosition(Position position) {
        motor.setControl(new PositionDutyCycle(getTarget(position)));
    }

    public boolean isAtPosition(Position position) {
        motorPosition.refresh();
        return Math.abs(motorPosition.getValue() - getTarget(position)) < NTData.AMP_ARM_TOLERANCE.get() / 360.0;
    }

    private double getTarget(Position position) {
        return (switch (position) {
            case STOW -> NTData.AMP_ARM_STOW_ANGLE;
            case PICKUP -> NTData.AMP_ARM_PICKUP_ANGLE;
            case SCORE -> NTData.AMP_ARM_SCORE_ANGLE;
        }).get() / 360.0;
    }

    private double getPositionWithCanCoder() {
        encoderPosition.refresh();
        return encoderPosition.getValue() + NTData.AMP_ARM_CANCODER_OFFSET.get();
    }

    // Assumes arm is physically in STOW position
    private void calibrateCanCoder() {
        encoderPosition.refresh();
        double currentPosition = encoderPosition.getValue();
        double expectedPosition = getTarget(Position.STOW);

        NTData.AMP_ARM_CANCODER_OFFSET.set(expectedPosition - currentPosition);
    }

    @Override
    public void periodic() {
        if (NTData.AMP_ARM_CALIBRATE.get()) {
            NTData.AMP_ARM_CALIBRATE.set(false);
            calibrateCanCoder();
        }
    }

    @Override
    public void simulationPeriodic() {
        motor.updateSim(12);
        SimView.updateAmpArm(motor.getPosition().getValue());
    }
}
