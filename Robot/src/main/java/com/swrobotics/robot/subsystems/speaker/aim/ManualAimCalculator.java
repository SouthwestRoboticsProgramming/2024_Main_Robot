package com.swrobotics.robot.subsystems.speaker.aim;

import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTDouble;

public final class ManualAimCalculator implements AimCalculator {
    private static final NTDouble velocitySetpoint = new NTDouble("Shooter/Manual Aim/Velocity (rot per sec)", 0);
    private static final NTDouble pivotSetpoint = new NTDouble("Shooter/Manual Aim/Pivot Angle (deg)", 30);
    private static final NTDouble logDistance = new NTDouble("Shooter/Manual Aim/Est Distance (m)", 0);

    private static final NTBoolean setFromTable = new NTBoolean("Shooter/Manual Aim/Set From Table", false);

    private final TableAimCalculator tableRef = new TableAimCalculator();

    @Override
    public Aim calculateAim(double distanceToSpeaker) {
        logDistance.set(distanceToSpeaker);

        if (setFromTable.get()) {
//            setFromTable.set(false);
            Aim ref = tableRef.calculateAim(distanceToSpeaker);
            velocitySetpoint.set(ref.flywheelVelocity());
            pivotSetpoint.set(Math.toDegrees(ref.pivotAngle()));
        }

        return new Aim(
                velocitySetpoint.get(),
                Math.toRadians(pivotSetpoint.get()),
                distanceToSpeaker
        );
    }
}
