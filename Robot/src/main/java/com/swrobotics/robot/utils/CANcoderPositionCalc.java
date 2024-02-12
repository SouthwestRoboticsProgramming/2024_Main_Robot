package com.swrobotics.robot.utils;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.mathlib.MathUtil;

import java.util.function.Supplier;

public final class CANcoderPositionCalc {
    public static final class PositionOverlapResolver {
        private final double encoderToMechanismRatio;
        private final double wrapMin, wrapMax;

        public PositionOverlapResolver(double encoderToMechanismRatio, double wrapMin, double wrapMax) {
            this.encoderToMechanismRatio = encoderToMechanismRatio;
            this.wrapMin = wrapMin;
            this.wrapMax = wrapMax;
        }

        public double resolveAbsolutePos(double canCoderPos) {
            return MathUtil.wrap(canCoderPos, wrapMin, wrapMax) / encoderToMechanismRatio;
        }
    }

    private final CANcoder encoder;
    private final Supplier<Double> relativePositionGetter;
    private final NTEntry<Double> offset;

    private final StatusSignal<Double> encoderPos;
    private final PositionOverlapResolver overlapResolver;

    // Offset between absolute position and the motor's relative position
    // We use this instead of setting the motor's position because REV
    // cannot be trusted
    private double localOffset;

    public CANcoderPositionCalc(CANcoder encoder, Supplier<Double> relativePositionGetter, NTEntry<Double> offset) {
        this(encoder, relativePositionGetter, offset, new PositionOverlapResolver(1, 0, 1));
    }

    // Encoder should already be configured with correct sensor direction
    public CANcoderPositionCalc(
            CANcoder encoder,
            Supplier<Double> relativePositionGetter,
            NTEntry<Double> offset,
            PositionOverlapResolver overlapResolver) {
        this.encoder = encoder;
        this.relativePositionGetter = relativePositionGetter;
        this.offset = offset;
        this.overlapResolver = overlapResolver;

        encoderPos = encoder.getAbsolutePosition();

        seed();
    }

    private double getAbsoluteEncoderPos() {
        encoderPos.refresh();
        return overlapResolver.resolveAbsolutePos(encoderPos.getValue());
    }

    public void seed() {
        localOffset = getAbsoluteEncoderPos() + offset.get() // Assume encoder to mechanism ratio is 1:1
                - relativePositionGetter.get();
    }

    public double getAbsolute() {
        return relativePositionGetter.get() + localOffset;
    }

    public double getRelativeForAbsolute(double absolutePosition) {
        return absolutePosition - localOffset;
    }

    public void calibrateCanCoder(double knownAbsolutePosition) {
        double currentPosition = getAbsoluteEncoderPos();
        offset.set(knownAbsolutePosition - currentPosition);
        seed();
    }
}
