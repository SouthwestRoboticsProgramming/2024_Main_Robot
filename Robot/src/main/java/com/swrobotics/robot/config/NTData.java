package com.swrobotics.robot.config;

import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;

public final class NTData {
    public static final NTEntry<Double> DRIVE_SPEED_SLOW = new NTDouble("Drive/Slow Speed", 1.5).setPersistent();
    public static final NTEntry<Double> DRIVE_SPEED_NORMAL = new NTDouble("Drive/Normal Speed", 3).setPersistent();
    public static final NTEntry<Double> DRIVE_SPEED_FAST = new NTDouble("Drive/Fast Speed", SwerveDrive.MAX_LINEAR_SPEED).setPersistent();
    public static final NTEntry<Double> TURN_SPEED = new NTDouble("Drive/Turn Speed (rot/s)", 2).setPersistent();

    public static final NTEntry<Double> DRIVE_AIM_KP = new NTDouble("Drive/Aim/PID/kP", 6).setPersistent();
    public static final NTEntry<Double> DRIVE_AIM_KI = new NTDouble("Drive/Aim/PID/kI", 0).setPersistent();
    public static final NTEntry<Double> DRIVE_AIM_KD = new NTDouble("Drive/Aim/PID/kD", 0).setPersistent();
    public static final NTEntry<Double> DRIVE_AIM_MAX_TURN = new NTDouble("Drive/Aim/Max Turn Speed (rot/s)", 2).setPersistent();

    public static final NTEntry<Double> DRIVE_FL_OFFSET = new NTDouble("Drive/Modules/Front Left Offset", 0).setPersistent();
    public static final NTEntry<Double> DRIVE_FR_OFFSET = new NTDouble("Drive/Modules/Front Right Offset", 0).setPersistent();
    public static final NTEntry<Double> DRIVE_BL_OFFSET = new NTDouble("Drive/Modules/Back Left Offset", 0).setPersistent();
    public static final NTEntry<Double> DRIVE_BR_OFFSET = new NTDouble("Drive/Modules/Back Right Offset", 0).setPersistent();

    public static final NTEntry<Double> INTAKE_RANGE = new NTDouble("Intake/Range (degrees)", 90).setPersistent();
    public static final NTEntry<Double> INTAKE_SPEED = new NTDouble("Intake/Speed", 0.3).setPersistent();
    public static final NTEntry<Double> INTAKE_EJECT_SPEED = new NTDouble("Intake/Eject Speed", 0.5).setPersistent();
    public static final NTEntry<Double> INTAKE_CALIBRATE_VOLTS = new NTDouble("Intake/Calibration/Output (volts)", 4).setPersistent();
    public static final NTEntry<Double> INTAKE_CALIBRATE_DEBOUNCE = new NTDouble("Intake/Calibration/Debounce Time (secs)", 0.5).setPersistent();
    public static final NTEntry<Double> INTAKE_CALIBRATE_STALL_THRESHOLD = new NTDouble("Intake/Calibration/Stall Threshold (rot/s)", 0.2).setPersistent();
    public static final NTEntry<Boolean> INTAKE_RECALIBRATE = new NTBoolean("Intake/Calibration/Recalibrate", false);
    public static final NTEntry<Double> INTAKE_KP = new NTDouble("Intake/PID/kP", 0).setPersistent();
    public static final NTEntry<Double> INTAKE_KI = new NTDouble("Intake/PID/kI", 0).setPersistent();
    public static final NTEntry<Double> INTAKE_KD = new NTDouble("Intake/PID/kD", 0).setPersistent();

    public static final NTEntry<Double> CLIMBER_EXTEND_SPEED = new NTDouble("Climber/Extend Speed", 0.8).setPersistent();
    public static final NTEntry<Double> CLIMBER_RETRACT_SPEED = new NTDouble("Climber/Retract Speed", 0.5).setPersistent();

    public static final NTEntry<Boolean> AMP_ARM_CALIBRATE = new NTBoolean("Amp/Arm/Calibrate (put in STOW)", false);
    public static final NTEntry<Double> AMP_ARM_CANCODER_OFFSET = new NTDouble("Amp/Arm/CanCoder Offset", 0).setPersistent();
    public static final NTEntry<Double> AMP_ARM_TOLERANCE = new NTDouble("Amp/Arm/Tolerance (degrees)", 1.5).setPersistent();
    // Angle of bottom segment relative to horizontal
    public static final NTEntry<Double> AMP_ARM_STOW_ANGLE = new NTDouble("Amp/Arm/Stow Angle (degrees)", 135).setPersistent();
    public static final NTEntry<Double> AMP_ARM_PICKUP_ANGLE = new NTDouble("Amp/Arm/Pickup Angle (degrees)", -45).setPersistent();
    public static final NTEntry<Double> AMP_ARM_SCORE_ANGLE = new NTDouble("Amp/Arm/Score Angle (degrees)", 45).setPersistent();
    public static final NTEntry<Double> AMP_ARM_KP = new NTDouble("Amp/Arm/PID/kP", 0.05).setPersistent();
    public static final NTEntry<Double> AMP_ARM_KI = new NTDouble("Amp/Arm/PID/kI", 0).setPersistent();
    public static final NTEntry<Double> AMP_ARM_KD = new NTDouble("Amp/Arm/PID/kD", 0).setPersistent();

    public static final NTEntry<Double> AMP_INTAKE_INTAKE_SPEED = new NTDouble("Amp/Intake/Intake Speed", 0.5).setPersistent();
    public static final NTEntry<Double> AMP_INTAKE_OUTTAKE_SPEED = new NTDouble("Amp/Intake/Outtake Speed", 0.5).setPersistent();

    public static final NTEntry<Double> INDEXER_SIDES_TAKE_SPEED = new NTDouble("Indexer/Sides/Take Speed", 0.2).setPersistent();
    public static final NTEntry<Double> INDEXER_SIDES_FEED_SPEED = new NTDouble("Indexer/Sides/Feed Speed", 0.6).setPersistent();
    public static final NTEntry<Double> INDEXER_SIDES_EJECT_SPEED = new NTDouble("Indexer/Sides/Eject Speed", 0.4).setPersistent();
    public static final NTEntry<Double> INDEXER_TOP_TAKE_SPEED = new NTDouble("Indexer/Top/Take Speed", 0.3).setPersistent();
    public static final NTEntry<Double> INDEXER_TOP_FEED_SPEED = new NTDouble("Indexer/Top/Feed Speed", 0.6).setPersistent();
    public static final NTEntry<Double> INDEXER_TOP_EJECT_SPEED = new NTDouble("Indexer/Top/Eject Speed", 0.4).setPersistent();

    public static final NTEntry<Double> SHOOTER_IDLE_SPEED = new NTDouble("Shooter/Idle Speed", 0.1).setPersistent();
    public static final NTEntry<Double> SHOOTER_FULL_SPEED_DISTANCE = new NTDouble("Shooter/Full Speed Distance (m)", 6).setPersistent();
    public static final NTEntry<Double> SHOOTER_AFTER_DELAY = new NTDouble("Shooter/After Shoot Delay (s)", 1).setPersistent();
    public static final NTEntry<Double> SHOOTER_ALLOWABLE_PCT_ERR = new NTDouble("Shooter/Allowable Pct Error", 0.05).setPersistent();
    public static final NTEntry<Double> SHOOTER_FLYWHEEL_KP = new NTDouble("Shooter/Flywheel PIDV/kP", 0).setPersistent();
    public static final NTEntry<Double> SHOOTER_FLYWHEEL_KI = new NTDouble("Shooter/Flywheel PIDV/kI", 0).setPersistent();
    public static final NTEntry<Double> SHOOTER_FLYWHEEL_KD = new NTDouble("Shooter/Flywheel PIDV/kD", 0).setPersistent();
    public static final NTEntry<Double> SHOOTER_FLYWHEEL_KV = new NTDouble("Shooter/Flywheel PIDV/kV", 0).setPersistent();
    public static final NTEntry<Boolean> SHOOTER_PIVOT_CALIBRATE = new NTBoolean("Shooter/Calibrate (put fully down)", false).setPersistent();
    public static final NTEntry<Double> SHOOTER_PIVOT_CANCODER_OFFSET = new NTDouble("Shooter/Pivot CanCoder Offset", 0).setPersistent();
    public static final NTEntry<Double> SHOOTER_PIVOT_IDLE_ANGLE = new NTDouble("Shooter/Pivot Idle Angle (degrees)", 35).setPersistent();
    public static final NTEntry<Double> SHOOTER_PIVOT_KP = new NTDouble("Shooter/Pivot PID/kP", 0).setPersistent();
    public static final NTEntry<Double> SHOOTER_PIVOT_KI = new NTDouble("Shooter/Pivot PID/kI", 0).setPersistent();
    public static final NTEntry<Double> SHOOTER_PIVOT_KD = new NTDouble("Shooter/Pivot PID/kD", 0).setPersistent();

    private NTData() {
        throw new AssertionError();
    }
}
