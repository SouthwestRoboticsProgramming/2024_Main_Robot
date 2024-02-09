package com.swrobotics.robot.utils;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.swrobotics.lib.net.NTEntry;

import java.util.function.Supplier;

public final class CANcoderSeededPosition {
    private final CANcoder encoder;
    private final Supplier<Double> relativePositionGetter;
    private final NTEntry<Double> offset;

    private final StatusSignal<Double> encoderPos;

    // Offset between absolute position and the motor's relative position
    // We use this instead of setting the motor's position because REV
    // cannot be trusted
    private double localOffset;

    // Encoder should already be configured with correct sensor direction
    public CANcoderSeededPosition(
            CANcoder encoder,
            Supplier<Double> relativePositionGetter,
            NTEntry<Double> offset) {
        this.encoder = encoder;
        this.relativePositionGetter = relativePositionGetter;
        this.offset = offset;

        encoderPos = encoder.getAbsolutePosition();

        seed();
    }

    private void setMotorPosition(double position) {
        double currentPos = relativePositionGetter.get();
        localOffset = position - currentPos;
    }

    public void seed() {
        encoderPos.refresh();
        localOffset = encoderPos.getValue() + offset.get() // Assume encoder to mechanism ratio is 1:1
                - relativePositionGetter.get();
    }

    public double getAbsolute() {
        return relativePositionGetter.get() + localOffset;
    }

    public double getRelativeForAbsolute(double absolutePosition) {
        return absolutePosition - localOffset;
    }

    public void calibrateCanCoder(double knownAbsolutePosition) {
        encoderPos.refresh();
        // Assume encoder to mechanism ratio is 1:1
        double currentPosition = encoderPos.getValue();

        offset.set(knownAbsolutePosition - currentPosition);
        seed();
    }
}
