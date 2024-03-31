package com.swrobotics.robot.subsystems.speaker.aim;

import com.swrobotics.robot.config.NTData;

public class AmpAimCalculator implements AimCalculator {
    public static final AmpAimCalculator INSTANCE = new AmpAimCalculator();

    @Override
    public Aim calculateAim(double distanceToSpeaker) {
        // Always returns the same aim regardless of distance to speaker
        return new Aim(
            NTData.SHOOTER_AMP_VELOCITY.get(),
            Math.toRadians(NTData.SHOOTER_AMP_ANGLE.get()),
            distanceToSpeaker);
    }
}
