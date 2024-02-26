package com.swrobotics.robot.subsystems.speaker.aim;

import com.swrobotics.lib.net.NTDouble;

public class AmpAimCalculator implements AimCalculator {
    private static final NTDouble velocitySetpoint = new NTDouble("Shooter/Amp/Velocity (rot per sec)", 30);
    private static final NTDouble pivotSetpoint = new NTDouble("Shooter/Amp/Pivot Angle (deg)", 65);

    @Override
    public Aim calculateAim(double distanceToSpeaker) {
        // Always returns the same aim regardless of distance to speaker
        return new Aim(
            velocitySetpoint.get(),
            pivotSetpoint.get());
    }
    
}
