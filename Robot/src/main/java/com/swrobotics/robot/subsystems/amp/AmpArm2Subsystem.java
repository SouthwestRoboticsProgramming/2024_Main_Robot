package com.swrobotics.robot.subsystems.amp;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class AmpArm2Subsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(IOAllocation.CAN.AMP_ARM_MOTOR.id(), IOAllocation.CAN.AMP_ARM_MOTOR.bus());
    private final CANcoder encoder = new CANcoder(IOAllocation.CAN.AMP_ARM_CANCODER.id(), IOAllocation.CAN.AMP_ARM_CANCODER.bus());

    public static final NTEntry<Double> currentPosition = new NTDouble("Amp Arm Test/Current position (deg)", 0);

    private final StatusSignal<Double> encoderPosition;
    private final StatusSignal<Double> motorPosition;

    private static final double motorToArmRatio = 50;
    private static final double encoderToArmRatio = 2;

    private static final double cancoderOffset = 0.045410;

    private Position targetPos;

    public enum Position {
        RETRACT(NTData.AMP_ARM_RETRACT_POS),
        AMP(NTData.AMP_ARM_EXTEND_POS),
        CLIMB_OUT_OF_THE_WAY(NTData.AMP_ARM_OUT_OF_THE_WAY_POS);

        final NTEntry<Double> positionNt;

        Position(NTEntry<Double> positionNt) {
            this.positionNt = positionNt;
        }
    }

    public AmpArm2Subsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = NTData.AMP_ARM_KP.get();
        config.Slot0.kD = NTData.AMP_ARM_KD.get();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = motorToArmRatio;
        motor.getConfigurator().apply(config);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = 0;
        encoder.getConfigurator().apply(encoderConfig);

        encoderPosition = encoder.getAbsolutePosition();
        encoderPosition.refresh();
        double encoderRangeCenter = (NTData.AMP_ARM_EXTEND_POS.get() / 360.0 + NTData.AMP_ARM_RETRACT_POS.get() / 360.0) / 2;
        double pos = MathUtil.wrap((encoderPosition.getValue() + cancoderOffset) / encoderToArmRatio, encoderRangeCenter - 0.25, encoderRangeCenter + 0.25);
        motor.setPosition(pos);

        motorPosition = motor.getPosition();

        targetPos = Position.RETRACT;

        NTData.AMP_ARM_KP.onChange((p) -> updatePD());
        NTData.AMP_ARM_KD.onChange((d) -> updatePD());
    }

    private void updatePD() {
        motor.getConfigurator().apply(new Slot0Configs()
        .withKP(NTData.AMP_ARM_KP.get())
        .withKD(NTData.AMP_ARM_KD.get()));
    }

    // public void setOut(boolean out) {
    //     targetPos = out ? NTData.AMP_ARM_2_EXTEND_POS.get() / 360.0 : retractPos;
    // }

    public void setPosition(Position pos) {
        targetPos = pos;
    }

    NTDouble d = new NTDouble("aaaaaaaaaaaaaaa", 0);

    private double calcFeedforwardVolts(double armAngle) {
        return NTData.AMP_ARM_GRAVITY_AMOUNT.get() * Math.cos(armAngle * MathUtil.TAU);
    }

    @Override
    public void periodic() {
        encoderPosition.refresh();
        motorPosition.refresh();
        motor.setControl(new PositionVoltage(targetPos.positionNt.get() / 360.0).withFeedForward(calcFeedforwardVolts(motorPosition.getValue())));
        currentPosition.set(motorPosition.getValue() * 360);
        d.set(encoderPosition.getValue() * 360);
    }

    public TalonFX getMotor() {
        return motor;
    }
}
