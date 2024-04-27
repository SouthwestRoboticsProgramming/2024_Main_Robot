package com.swrobotics.robot.subsystems.speaker.aim;

public class LowLobAimCalculator implements AimCalculator {
    public static final LowLobAimCalculator INSTANCE = new LowLobAimCalculator();

    @Override
    public Aim calculateAim(double distanceToSpeaker) {
        return new Aim(30, Math.toRadians(25), distanceToSpeaker);
    }
    
}
