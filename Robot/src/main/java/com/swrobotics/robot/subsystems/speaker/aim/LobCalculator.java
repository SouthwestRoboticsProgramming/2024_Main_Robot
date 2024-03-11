package com.swrobotics.robot.subsystems.speaker.aim;

import com.swrobotics.lib.net.NTDouble;

public class LobCalculator implements AimCalculator {
    public static final LobCalculator INSTANCE = new LobCalculator();

    private NTDouble LOB_HEIGHT;
    private NTDouble LOB_POWER_MULTIPLIER;
    private static final double twoG = 9.8 * 2;
    private double sqrt2gh;

    public LobCalculator() {
        LOB_HEIGHT = new NTDouble("Shooter/Lob/Height (m)", 3);
        LOB_POWER_MULTIPLIER = new NTDouble("Shooter/Lob/Power Multiplier", 1.25);
        LOB_HEIGHT.onChange((a) -> this.updateHeight());
        sqrt2gh = Math.sqrt(twoG * LOB_HEIGHT.get());
    }

    @Override
    public Aim calculateAim(double distanceToSpeaker) {
        double angleRad = Math.atan2(4 * LOB_HEIGHT.get(), distanceToSpeaker);
        double velocity = sqrt2gh / Math.sin(angleRad);
        double velocitySetpoint = velocity * LOB_POWER_MULTIPLIER.get();

        System.out.println("V: " + velocitySetpoint + " A: " + Math.toDegrees(angleRad));
        return new Aim(velocitySetpoint, angleRad);
    }

    private void updateHeight() {
        sqrt2gh = Math.sqrt(twoG * LOB_HEIGHT.get());
    }
    
}
