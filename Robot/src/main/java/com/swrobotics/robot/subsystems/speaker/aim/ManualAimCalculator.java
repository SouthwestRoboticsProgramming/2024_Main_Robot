package com.swrobotics.robot.subsystems.speaker.aim;

import com.swrobotics.lib.net.NTDouble;

public final class ManualAimCalculator implements AimCalculator {
    private static final NTDouble velocitySetpoint = new NTDouble("Shooter/Manual Aim/Velocity (rot per sec)", 0);
    private static final NTDouble pivotSetpoint = new NTDouble("Shooter/Manual Aim/Pivot Angle (deg)", 30);
    private static final NTDouble logDistance = new NTDouble("Shooter/Manual Aim/Est Distance (m)", 0);

    @Override
    public Aim calculateAim(double distanceToSpeaker) {
        logDistance.set(distanceToSpeaker);

        return new Aim(
                velocitySetpoint.get(),
                Math.toRadians(pivotSetpoint.get())
        );
    }
}
