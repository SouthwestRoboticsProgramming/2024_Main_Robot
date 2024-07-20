package com.swrobotics.robot.subsystems.speaker.aim;

import java.util.function.Supplier;

import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.NTData;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class LobCalculator implements AimCalculator {
    public static final LobCalculator INSTANCE = new LobCalculator();

    private static final double twoG = 9.8 * 2;
    private double sqrt2gh;
    private double flytime;

    private NTEntry<Double> aimOffset;

    public LobCalculator() {
        NTData.SHOOTER_LOB_HEIGHT_METERS.onChange((a) -> this.updateHeight());
        updateHeight();


    }

    public Aim calculateAim(double distanceToTarget, double velocityTowardsGoal) {

        double angleRad = Math.atan2(4 * NTData.SHOOTER_LOB_HEIGHT_METERS.get(), distanceToTarget);

        double velocity = sqrt2gh / Math.sin(angleRad);

        Translation2d velocityVector = new Translation2d(velocity, new Rotation2d(angleRad));
        Translation2d driveVelocityVector = new Translation2d(velocityTowardsGoal, 0);
        Translation2d shooterVelocity = velocityVector.minus(driveVelocityVector);

        double velocitySetpoint = shooterVelocity.getNorm() * NTData.SHOOTER_LOB_POWER_COEFFICIENT.get();

        // System.out.println("V: " + velocitySetpoint + " A: " + Math.toDegrees(angleRad));
        return new Aim(velocitySetpoint, MathUtil.clamp(shooterVelocity.getAngle().getRadians(), Math.toRadians(23), Math.toRadians(69)), distanceToTarget);
    }

    @Override
    public Aim calculateAim(double distanceToTarget) {
        return calculateAim(distanceToTarget, 0);
    }

    private void updateHeight() {
        sqrt2gh = Math.sqrt(twoG * NTData.SHOOTER_LOB_HEIGHT_METERS.get());
        flytime = 2 * Math.sqrt(2 * NTData.SHOOTER_LOB_HEIGHT_METERS.get() / 9.8);
    }

    public double getFlyTime() {
        return flytime;
    }

    public double getAimOffset() {

        // For some reason it's different per alliance???
        NTEntry<Double> entry = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? NTData.SHOOTER_LOB_DRIVE_ANGLE_CORRECTION_BLUE
                : NTData.SHOOTER_LOB_DRIVE_ANGLE_CORRECTION_RED;
        return entry.get();
    }

}
