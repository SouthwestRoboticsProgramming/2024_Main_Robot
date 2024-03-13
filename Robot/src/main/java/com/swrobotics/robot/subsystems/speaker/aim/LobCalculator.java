package com.swrobotics.robot.subsystems.speaker.aim;

import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.NTData;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class LobCalculator implements AimCalculator {
    public static final LobCalculator INSTANCE = new LobCalculator();

    private static final double twoG = 9.8 * 2;
    private double sqrt2gh;

    public LobCalculator() {
        NTData.SHOOTER_LOB_HEIGHT_METERS.onChange((a) -> this.updateHeight());
        sqrt2gh = Math.sqrt(twoG * NTData.SHOOTER_LOB_HEIGHT_METERS.get());
    }

    public Aim calculateAim(double distanceToSpeaker, double velocityTowardsGoal) {
        double angleRad = Math.atan2(4 * NTData.SHOOTER_LOB_HEIGHT_METERS.get(), distanceToSpeaker);
        double velocity = sqrt2gh / Math.sin(angleRad);

        Translation2d velocityVector = new Translation2d(velocity, new Rotation2d(angleRad));
        Translation2d driveVelocityVector = new Translation2d(velocityTowardsGoal, 0);
        Translation2d shooterVelocity = velocityVector.minus(driveVelocityVector);

        double velocitySetpoint = shooterVelocity.getNorm() * NTData.SHOOTER_LOB_POWER_COEFFICIENT.get();

        // System.out.println("V: " + velocitySetpoint + " A: " + Math.toDegrees(angleRad));
        return new Aim(velocitySetpoint, MathUtil.clamp(shooterVelocity.getAngle().getRadians(), Math.toRadians(23), Math.toRadians(69)), distanceToSpeaker);
    }

    @Override
    public Aim calculateAim(double distanceToSpeaker) {
        return calculateAim(distanceToSpeaker, 0);
    }

    private void updateHeight() {
        sqrt2gh = Math.sqrt(twoG * NTData.SHOOTER_LOB_HEIGHT_METERS.get());
    }
    
}
