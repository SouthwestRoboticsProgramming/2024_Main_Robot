package com.swrobotics.robot.config;

import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;

public final class NTData {
    public static final NTEntry<Double> TURN_SPEED = new NTDouble("Drive/Turn Speed (rot per sec)", 1).setPersistent();

    public static final NTEntry<Double> DRIVE_AIM_KP = new NTDouble("Drive/Aim/PID/kP", 9).setPersistent();
    public static final NTEntry<Double> DRIVE_AIM_KD = new NTDouble("Drive/Aim/PID/kD", 0.5).setPersistent();
    public static final NTEntry<Double> DRIVE_AIM_MAX_TURN = new NTDouble("Drive/Aim/Max Turn Speed (rot per sec)", 1).setPersistent();
    public static final NTEntry<Double> DRIVE_AIM_TOLERANCE = new NTDouble("Drive/Aim/Tolerance (rot)", 0.05).setPersistent();
    public static final NTEntry<Double> DRIVE_AIM_OFFSET = new NTDouble("Drive/Aim/Offset (ccw deg)", -1.5).setPersistent();

    public static final NTEntry<Double> DRIVE_SNAP_KP = new NTDouble("Drive/Snap/PID/kP", 8).setPersistent();
    public static final NTEntry<Double> DRIVE_SNAP_KD = new NTDouble("Drive/Snap/PID/kD", 0).setPersistent();
    public static final NTEntry<Double> DRIVE_SNAP_MAX_SPEED = new NTDouble("Drive/Snap/PID/Max Speed (meters per sec)", 2).setPersistent();
    public static final NTEntry<Double> DRIVE_SNAP_DEADBAND = new NTDouble("Drive/Snap/PID/Deadband (m)", 0.05);

    public static final NTEntry<Boolean> DRIVE_CALIBRATE = new NTBoolean("Drive/Modules/Calibrate", false);
    public static final NTEntry<Double> DRIVE_FL_OFFSET = new NTDouble("Drive/Modules/Front Left Offset (rot)", 0.611084).setPersistent();
    public static final NTEntry<Double> DRIVE_FR_OFFSET = new NTDouble("Drive/Modules/Front Right Offset (rot)", 0.483154).setPersistent();
    public static final NTEntry<Double> DRIVE_BL_OFFSET = new NTDouble("Drive/Modules/Back Left Offset (rot)", -0.623535).setPersistent();
    public static final NTEntry<Double> DRIVE_BR_OFFSET = new NTDouble("Drive/Modules/Back Right Offset (rot)", 1.345947).setPersistent();

    public static final NTEntry<Double> INTAKE_RANGE = new NTDouble("Intake/Range (deg)", 111).setPersistent();
    public static final NTEntry<Double> INTAKE_SPEED = new NTDouble("Intake/Intake Speed (pct)", 0.4).setPersistent();
    public static final NTEntry<Double> INTAKE_CALIBRATE_SETPOINT = new NTDouble("Intake/Calibration/Setpoint (deg)", 0).setPersistent();
    public static final NTEntry<Boolean> INTAKE_RECALIBRATE = new NTBoolean("Intake/Calibration/Recalibrate", false);
    public static final NTEntry<Double> INTAKE_KP = new NTDouble("Intake/PID/kP", 0.06).setPersistent();
    public static final NTEntry<Double> INTAKE_KD = new NTDouble("Intake/PID/kD", 0).setPersistent();

    public static final NTEntry<Double> CLIMBER_EXTEND_POSITION = new NTDouble("Climber/Extended Position (rotor rot)", 75).setPersistent();
    public static final NTEntry<Double> CLIMBER_KP = new NTDouble("Climber/PID/kP", 0.25).setPersistent();
    public static final NTEntry<Double> CLIMBER_KD = new NTDouble("Climber/PID/kD", 0).setPersistent();
    public static final NTEntry<Double> CLIMBER_CALIBRATE_VOLTS = new NTDouble("Climber/Calibrate/Volts", 1).setPersistent();
    public static final NTEntry<Double> CLIMBER_CALIBRATE_TIME = new NTDouble("Climber/Calibrate/Time (s)", 0.25).setPersistent();
    public static final NTEntry<Double> CLIMBER_CALIBRATE_THRESHOLD = new NTDouble("Climber/Calibrate/Threshold (rot per sec)", 0.5).setPersistent();
    public static final NTEntry<Double> CLIMBER_CALIBRATE_POSITION = new NTDouble("Climber/Calibrate/Position (rotor rot)", -3).setPersistent();
    public static final NTEntry<Boolean> CLIMBER_RECALIBRATE = new NTBoolean("Climber/Calibrate/Recalibrate", false);
    public static final NTEntry<Boolean> CLIMBER_CALIBRATING = new NTBoolean("Climber/Is Calibrating", false);

    public static final NTEntry<Double> AMP_ARM_KP = new NTDouble("Amp Arm/kP", 65).setPersistent();
    public static final NTEntry<Double> AMP_ARM_KD = new NTDouble("Amp Arm/kD", 0).setPersistent();
    public static final NTEntry<Double> AMP_ARM_KV = new NTDouble("Amp Arm/kV", 0).setPersistent();
    public static final NTEntry<Double> AMP_ARM_KA = new NTDouble("Amp Arm/kA", 0).setPersistent();
    public static final NTEntry<Double> AMP_ARM_MM_CRUISE_VELOCITY = new NTDouble("Amp Arm/Motion Magic Cruise Velocity", 400).setPersistent();
    public static final NTEntry<Double> AMP_ARM_MM_ACCEL = new NTDouble("Amp Arm/Motion Magic Acceleration", 30).setPersistent();
    public static final NTEntry<Double> AMP_ARM_MM_JERK = new NTDouble("Amp Arm/Motion Magic Jerk", 100000000).setPersistent();

    public static final NTEntry<Double> AMP_ARM_RETRACT_POS = new NTDouble("Amp Arm/Retract Pos (deg)", 6).setPersistent();
    public static final NTEntry<Double> AMP_ARM_EXTEND_POS = new NTDouble("Amp Arm/Extend Pos (deg)", 114.780953).setPersistent();
    public static final NTEntry<Double> AMP_ARM_OUT_OF_THE_WAY_POS = new NTDouble("Amp Arm/Climb Out of the Way Pos (deg)", 90).setPersistent();
    public static final NTEntry<Double> AMP_ARM_GRAVITY_AMOUNT = new NTDouble("Amp Arm/Gravity (volts at horiz)", 0).setPersistent();
    public static final NTEntry<Double> AMP_ARM_PEAK_VOLTS = new NTDouble("Amp Arm/Peak Output Voltage", 12).setPersistent();
    public static final NTEntry<Double> AMP_ARM_CURRENT_LIMIT = new NTDouble("Amp Arm/Current Limit (A)", 16).setPersistent();

    public static final NTEntry<Double> TRAP_FINGER_HOLD_ANGLE = new NTDouble("Trap Finger/Hold Angle (deg)", 0).setPersistent();
    public static final NTEntry<Double> TRAP_FINGER_RELEASE_ANGLE = new NTDouble("Trap Finger/Release Angle (deg)", 90).setPersistent();

    public static final NTEntry<Double> INDEXER_TOP_TAKE_SPEED = new NTDouble("Indexer/Top/Take Speed (pct)", 0.7).setPersistent();
    public static final NTEntry<Double> INDEXER_TOP_FEED_SPEED = new NTDouble("Indexer/Top/Feed Speed (pct)", 0.6).setPersistent();
    public static final NTEntry<Double> INDEXER_TOP_INDEX_SPEED = new NTDouble("Indexer/Top/Index Speed (pct)", 0.4).setPersistent();
    public static final NTEntry<Double> INDEXER_FEED_ADDITIONAL_TIME = new NTDouble("Indexer/Feed Additional Time (sec)", 0.0).setPersistent();
    public static final NTEntry<Boolean> INDEXER_HAS_PIECE = new NTBoolean("Indexer/Has Piece", false);

    public static final NTEntry<Double> SHOOTER_AFTER_DELAY = new NTDouble("Shooter/After Shoot Delay (sec)", 1).setPersistent();
    public static final NTEntry<Boolean> SHOOTER_READY = new NTBoolean("Shooter/Is Ready", false); // Read by ShuffleLog
    public static final NTEntry<Double> SHOOTER_AUTO_READY_TIMEOUT = new NTDouble("Shooter/Auto/Ready Timeout (sec)", 0.25).setPersistent();
    public static final NTEntry<Double> SHOOTER_AUTO_AFTER_READY_DELAY = new NTDouble("Shooter/Auto/After Ready Delay (sec)", 0.15).setPersistent();
    public static final NTEntry<Double> SHOOTER_DISTANCE_SCALE = new NTDouble("Shooter/Distance Scale (pct)", 1).setPersistent();

    public static final NTEntry<Double> SHOOTER_AMP_VELOCITY = new NTDouble("Shooter/Amp/Velocity (rot per sec)", 27).setPersistent();
    public static final NTEntry<Double> SHOOTER_AMP_ANGLE = new NTDouble("Shooter/Amp/Pivot Angle (deg)", 50).setPersistent();

    public static final NTEntry<Double> SHOOTER_FLYWHEEL_IDLE_SPEED = new NTDouble("Shooter/Flywheel/Idle Speed (pct)", 0.0).setPersistent();
    public static final NTEntry<Double> SHOOTER_FLYWHEEL_POOP_SPEED = new NTDouble("Shooter/Flywheel/Poop Speed (pct) ", 0.3).setPersistent();
    public static final NTEntry<Double> SHOOTER_FLYWHEEL_REVERSE_SPEED = new NTDouble("Shooter/Flywheel/Reverse Speed (pct)", 0.3).setPersistent();
    public static final NTEntry<Double> SHOOTER_FLYWHEEL_ALLOWABLE_PCT_ERR = new NTDouble("Shooter/Flywheel/Allowable Pct Error", 0.02).setPersistent();
    public static final NTEntry<Double> SHOOTER_FLYWHEEL_KP = new NTDouble("Shooter/Flywheel/PIDV/kP", 0.3).setPersistent();
    public static final NTEntry<Double> SHOOTER_FLYWHEEL_KD = new NTDouble("Shooter/Flywheel/PIDV/kD", 0).setPersistent();
    public static final NTEntry<Double> SHOOTER_FLYWHEEL_KV = new NTDouble("Shooter/Flywheel/PIDV/kV", 0.126).setPersistent();

    public static final NTEntry<Double> SHOOTER_PIVOT_IDLE_ANGLE = new NTDouble("Shooter/Pivot/Idle Angle (deg)", 35).setPersistent();
    public static final NTEntry<Double> SHOOTER_PIVOT_KP = new NTDouble("Shooter/Pivot/PID/kP", 600).setPersistent();
    public static final NTEntry<Double> SHOOTER_PIVOT_KD = new NTDouble("Shooter/Pivot/PID/kD", 0).setPersistent();
    public static final NTEntry<Double> SHOOTER_PIVOT_KV = new NTDouble("Shooter/Pivot/PID/kV", 0).setPersistent();
    public static final NTEntry<Boolean> SHOOTER_PIVOT_RECALIBRATE = new NTBoolean("Shooter/Pivot/Calibration/Recalibrate", false);
    public static final NTEntry<Double> SHOOTER_PIVOT_CALIBRATE_VOLTS = new NTDouble("Shooter/Pivot/Calibration/Applied Volts", 1).setPersistent();
//    public static final NTEntry<Double> SHOOTER_PIVOT_ALLOWABLE_PCT_ERR = new NTDouble("Shooter/Pivot/Allowable Pct Error", 10).setPersistent(); // FIXME TUNE
    public static final NTEntry<Double> SHOOTER_PIVOT_ALLOWABLE_ERR = new NTDouble("Shooter/Pivot/Allowable Error (deg)", 1).setPersistent();
    public static final NTEntry<Double> SHOOTER_PIVOT_ANGLE_ADJUST_BLUE = new NTDouble("Shooter/Pivot/Angle Adjust Blue (deg)", 0).setPersistent();
    public static final NTEntry<Double> SHOOTER_PIVOT_ANGLE_ADJUST_RED = new NTDouble("Shooter/Pivot/Angle Adjust Red (deg)", -0.75).setPersistent();
    public static final NTEntry<Double> SHOOTER_PIVOT_REVERSE_ANGLE = new NTDouble("Shooter/Pivot/Reverse Angle (deg)", 52).setPersistent();

    public static final NTEntry<Double> SHOOTER_LOB_POWER_COEFFICIENT = new NTDouble("Shooter/Lob/Power Coefficient", 1.75).setPersistent(); // To go from real velocity to flywheel velocity
    public static final NTEntry<Double> SHOOTER_LOB_TALL_HEIGHT_METERS = new NTDouble("Shooter/Lob/Tall Lob Height (m)", 5).setPersistent();
    public static final NTEntry<Double> SHOOTER_LOB_SHORT_HEIGHT_METERS = new NTDouble("Shooter/Lob/Short Lob Height (m)", 70).setPersistent();
    public static final NTEntry<Double> SHOOTER_LOB_DRIVE_ANGLE_CORRECTION_BLUE = new NTDouble("Shooter/Lob/Drive Angle Correction Blue (deg)", -15).setPersistent();
    public static final NTEntry<Double> SHOOTER_LOB_DRIVE_ANGLE_CORRECTION_RED = new NTDouble("Shooter/Lob/Drive Angle Correction Red (deg)", 0).setPersistent();
    public static final NTEntry<Double> SHOOTER_FLY_TIME = new NTDouble("Shooter/Fly Time (s)", 0.25).setPersistent();
    public static final NTEntry<Double> GREEN_FN_COEFFICIENT = new NTDouble("Shooter/Green FN Coefficient", 3.5).setPersistent();

    private NTData() {
        throw new AssertionError();
    }
}
