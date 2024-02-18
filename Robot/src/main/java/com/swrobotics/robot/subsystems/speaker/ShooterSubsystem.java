package com.swrobotics.robot.subsystems.speaker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import com.swrobotics.robot.utils.CANcoderPositionCalc;
import com.swrobotics.robot.utils.TalonFXWithSim;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ShooterSubsystem extends SubsystemBase {
    private static final Pose2d blueSpeakerPose = new Pose2d(0, 5.5475, new Rotation2d(0));
    private static final double motorToPivotRatio = 10;
    private static final double hardStopAngle = 18 / 360.0;

    private final SwerveDrive drive;
    private final IndexerSubsystem indexer;

    private static TalonFXWithSim createFlywheel(IOAllocation.CanId id) {
        return new TalonFXWithSim(
                id,
                DCMotor.getFalcon500Foc(1),
                1,
                0.001
        );
    }

    public final TalonFXWithSim flywheelMotor1 = createFlywheel(IOAllocation.CAN.SHOOTER_MOTOR_1);
    public final TalonFXWithSim flywheelMotor2 = createFlywheel(IOAllocation.CAN.SHOOTER_MOTOR_2);
    private final StatusSignal<Double> flywheelVelocity1, flywheelVelocity2;

    private final TalonFXWithSim pivotMotor = new TalonFXWithSim(
            IOAllocation.CAN.SHOOTER_PIVOT_MOTOR,
            DCMotor.getFalcon500Foc(1),
            motorToPivotRatio,
            0.01
    );
    private final StatusSignal<Double> pivotPosition, pivotVelocity;

    private boolean hasCalibrated;
    private Debouncer calibrationDebounce;

    private Debouncer afterShootDelay;
    private boolean isPreparing;

    private double targetVelocity;
    private AimCalculator.Aim targetAim;

    public ShooterSubsystem(SwerveDrive drive, IndexerSubsystem indexer) {
        this.drive = drive;
        this.indexer = indexer;

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        applyFlywheelPIDV(flywheelConfig.Slot0);
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flywheelMotor1.getConfigurator().apply(flywheelConfig);
        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flywheelMotor2.getConfigurator().apply(flywheelConfig);
        flywheelVelocity1 = flywheelMotor1.getVelocity();
        flywheelVelocity2 = flywheelMotor2.getVelocity();

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        applyPivotPID(pivotConfig.Slot0);
        pivotConfig.Feedback.SensorToMechanismRatio = motorToPivotRatio;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.PeakForwardDutyCycle = 0.5;
        pivotConfig.MotorOutput.PeakReverseDutyCycle = -0.5;
        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotPosition = pivotMotor.getPosition();
        pivotVelocity = pivotMotor.getVelocity();

        NTData.SHOOTER_AFTER_DELAY.nowAndOnChange((delay) -> afterShootDelay = new Debouncer(delay, Debouncer.DebounceType.kFalling));
        NTData.SHOOTER_FLYWHEEL_KP.onChange(this::updateFlywheelPIDV);
        NTData.SHOOTER_FLYWHEEL_KI.onChange(this::updateFlywheelPIDV);
        NTData.SHOOTER_FLYWHEEL_KD.onChange(this::updateFlywheelPIDV);
        NTData.SHOOTER_FLYWHEEL_KV.onChange(this::updateFlywheelPIDV);
        NTData.SHOOTER_PIVOT_KP.onChange(this::updatePivotPID);
        NTData.SHOOTER_PIVOT_KI.onChange(this::updatePivotPID);
        NTData.SHOOTER_PIVOT_KD.onChange(this::updatePivotPID);
        NTData.SHOOTER_PIVOT_KV.onChange(this::updatePivotPID);

        hasCalibrated = RobotBase.isSimulation();
        calibrationDebounce = null;
    }

    // TODO: Find a better way to do this (maybe TalonFXWithSim has helper?)
    private void applyFlywheelPIDV(Slot0Configs config) {
        config.kP = NTData.SHOOTER_FLYWHEEL_KP.get();
        config.kI = NTData.SHOOTER_FLYWHEEL_KI.get();
        config.kD = NTData.SHOOTER_FLYWHEEL_KD.get();
        config.kV = NTData.SHOOTER_FLYWHEEL_KV.get();
    }

    private void updateFlywheelPIDV(double ignored) {
        Slot0Configs configs = new Slot0Configs();
        applyFlywheelPIDV(configs);
        flywheelMotor1.getConfigurator().apply(configs);
        flywheelMotor2.getConfigurator().apply(configs);
    }

    private void applyPivotPID(Slot0Configs config) {
        config.kP = NTData.SHOOTER_PIVOT_KP.get();
        config.kI = NTData.SHOOTER_PIVOT_KI.get();
        config.kD = NTData.SHOOTER_PIVOT_KD.get();
        config.kV = NTData.SHOOTER_PIVOT_KV.get();
    }

    private void updatePivotPID(double ignored) {
        Slot0Configs configs = new Slot0Configs();
        applyPivotPID(configs);
        pivotMotor.getConfigurator().apply(configs);
    }

    public Translation2d getSpeakerPosition() {
        return drive.getFieldInfo().flipPoseForAlliance(blueSpeakerPose).getTranslation();
    }

    private void setPivotTarget(double angleRot) {
        if (Double.isNaN(angleRot) || !Double.isFinite(angleRot))
            angleRot = 0; // Make it finite
        angleRot = Math.max(angleRot, hardStopAngle + 3 / 360.0);
        System.out.println("Pivot control req: " + angleRot + ", curr: " + pivotPosition.getValue());
        pivotSetpoint.set(angleRot * 360);
        pivotCurrent.set(pivotPosition.getValue() * 360);
        pivotMotor.setControl(new PositionVoltage(angleRot));
    }

    private void setFlywheelTarget(double velocityRPS) {
        // TODO: Torque/FOC?

        targetVelocity = velocityRPS;
        flywheelMotor1.setControl(new VelocityVoltage(velocityRPS));
        flywheelMotor2.setControl(new VelocityVoltage(velocityRPS));
    }

    private double flywheelVelocityForShotVelocity(double shotVelocityMetersPerSec) {
        return shotVelocityMetersPerSec; // FIXME
//        return shotVelocityMetersPerSec / AimCalculator.velocity * 3000 / 60.0;
    }

    @Override
    public void periodic() {
//        flywheelMotor1.setControl(new NeutralOut());
//        flywheelMotor2.setControl(new NeutralOut());
//        pivotMotor.setControl(new DutyCycleOut(0.8));
//        if (true) return;

        StatusSignal.refreshAll(flywheelVelocity1, flywheelVelocity2, pivotPosition);
        if (DriverStation.isDisabled())
            return;

        if (NTData.SHOOTER_PIVOT_RECALIBRATE.get()) {
            NTData.SHOOTER_PIVOT_RECALIBRATE.set(false);
            hasCalibrated = false;
            calibrationDebounce = null;
        }

        if (!hasCalibrated) {
//            if (calibrationDebounce == null) {
//                calibrationDebounce = new Debouncer(NTData.SHOOTER_PIVOT_RECALIBRATE_DEBOUNCE.get(), Debouncer.DebounceType.kBoth);
//            }
//
//            pivotVelocity.refresh();
//            boolean isStill = Math.abs(pivotVelocity.getValue()) < NTData.SHOOTER_PIVOT_STALL_THRESHOLD.get();
//            if (calibrationDebounce.calculate(isStill)) {
                hasCalibrated = true;
//
                pivotMotor.setPosition(hardStopAngle);
//                pivotMotor.setControl(new NeutralOut());
//                NTData.SHOOTER_PIVOT_CALIBRATING.set(false);
//            } else {
//                pivotMotor.setControl(new VoltageOut(-NTData.SHOOTER_PIVOT_CALIBRATE_VOLTS.get()));
//                NTData.SHOOTER_PIVOT_CALIBRATING.set(true);
//            }
//
//            return;
        }

//        setFlywheelTarget(flywheelVelocityForShotVelocity(134891324));

        isPreparing = false;
        if (afterShootDelay.calculate(indexer.hasPiece())) {
            double distToSpeaker = getSpeakerPosition().getDistance(drive.getEstimatedPose().getTranslation());

            AimCalculator.Aim aim = AimCalculator.calculateAim(distToSpeaker);
            if (aim != null) {
                targetAim = aim;
                isPreparing = true;
                setFlywheelTarget(flywheelVelocityForShotVelocity(aim.flywheelVelocity()));
                setPivotTarget(aim.pivotAngle() / MathUtil.TAU);
            } else {
                targetAim = null;
                DutyCycleOut cmd = new DutyCycleOut(NTData.SHOOTER_IDLE_SPEED.get());
                flywheelMotor1.setControl(cmd);
                flywheelMotor2.setControl(cmd);
                setPivotTarget(NTData.SHOOTER_PIVOT_IDLE_ANGLE.get() / 360.0);
            }
        } else {
            targetAim = null;
            flywheelMotor1.setControl(new NeutralOut());
            flywheelMotor2.setControl(new NeutralOut());
            pivotMotor.setControl(new NeutralOut());
        }

        ready.set(isReadyToShoot());
        pctErr.set(getPctErr());
        velSetpoint.set(targetVelocity);
        velCurrent.set(flywheelVelocity1.getValue());
    }

    NTBoolean ready = new NTBoolean("Shooter/Debug/Flywheel Ready", false);
    NTDouble pctErr = new NTDouble("Shooter/Debug/Percent Error", 0);
    NTDouble velSetpoint = new NTDouble("Shooter/Debug/Velocity Setpoint", 0);
    NTDouble velCurrent = new NTDouble("Shooter/Debug/Velocity Current", 0);
    NTDouble pivotSetpoint = new NTDouble("Shooter/Debug/Pivot Setpoint (deg)", 0);
    NTDouble pivotCurrent = new NTDouble("Shooter/Debug/Pivot Current (deg)", 0);

    @Override
    public void simulationPeriodic() {
        flywheelMotor1.updateSim(12);
        flywheelMotor2.updateSim(12);
        pivotMotor.updateSim(12);
        SimView.updateShooter(pivotPosition.getValue());
        if (targetAim == null)
            SimView.targetTrajectory.clear();
        else
            SimView.targetTrajectory.update(targetAim);
    }

    public boolean isPreparing() {
        return isPreparing;
    }

    // For the status indicator in lights
    // <1 if too slow, >1 if too high
    public double getPercentOfTarget() {
        double min = Math.min(flywheelVelocity1.getValue(), flywheelVelocity2.getValue());
        return min / targetVelocity;
    }

    private double getPctErr() {
        double err1 = Math.abs(MathUtil.signedPercentError(flywheelVelocity1.getValue(), targetVelocity));
        double err2 = Math.abs(MathUtil.signedPercentError(flywheelVelocity2.getValue(), targetVelocity));
        return Math.min(err1, err2);
    }

    public boolean isReadyToShoot() {
        return isPreparing && getPctErr() < NTData.SHOOTER_ALLOWABLE_PCT_ERR.get();
    }
}
