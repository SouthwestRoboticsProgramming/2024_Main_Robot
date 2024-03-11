package com.swrobotics.robot.subsystems.speaker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.utils.TalonFXWithSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class PivotSubsystem extends SubsystemBase {
    /**
     * All possible sources of error:
     *
     * Vision estimates:
     *   [x] Tag is wrong size
     *   Camera calibration is wrong
     *   [/] Estimates are poor quality
     *
     * Tuning:
     *   The tuning was wrong
     *
     * Targeting:
     *   [/] Target point is wrong for the event field (field is not right dimensions)
     *
     * Shooting:
     *   [x] Different behavior between almost-new and used notes
     *   Pivot does not go to setpoint
     *   [x] Pivot does not calibrate to correct position
     *   [x] Flywheel does not go to setpoint
     *   [x] Indexer speed changed between shop and here
     *   Physics just behaves differently in Duluth than Minneapolis
     */

    public static NTEntry<Double> getAdjustForAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? NTData.SHOOTER_PIVOT_ANGLE_ADJUST_BLUE
                : NTData.SHOOTER_PIVOT_ANGLE_ADJUST_RED;
    }

    private enum State {
        CALIBRATING,
        IDLE,
        SHOOTING
    }

    private static final double motorToPivotRatio = 10 * 9 * 4;
    private static final double hardStopAngle = 22 / 360.0;
    private static final double maxAngle = (90 - 20) / 360.0;

    private final TalonFXWithSim motor = new TalonFXWithSim(
            IOAllocation.CAN.SHOOTER_PIVOT_MOTOR,
            DCMotor.getFalcon500Foc(1),
            motorToPivotRatio,
            0.01
    );
    private final StatusSignal<Double> position;
    private final StatusSignal<ReverseLimitValue> limitSwitch;

    private State state;
    private boolean calibrated;
    private double setpoint;

    public PivotSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        applyPID(config.Slot0);
        config.Feedback.SensorToMechanismRatio = motorToPivotRatio;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motor.getConfigurator().apply(config);
        position = motor.getPosition();
        limitSwitch = motor.getReverseLimit();

        NTData.SHOOTER_PIVOT_KP.onChange(this::updatePID);
        NTData.SHOOTER_PIVOT_KD.onChange(this::updatePID);
        NTData.SHOOTER_PIVOT_KV.onChange(this::updatePID);

        state = State.CALIBRATING;
        calibrated = false;

        setpoint = Double.NEGATIVE_INFINITY;
    }

    public void setTargetAngle(double angleRot) {
        if (state == State.CALIBRATING)
            return;

        angleRot += getAdjustForAlliance().get() / 360.0;

        angleRot = MathUtil.clamp(
                angleRot,
                hardStopAngle + 2 / 360.0,
                maxAngle - 2 / 360.0);

        motor.setControl(new PositionVoltage(angleRot));
        state = State.SHOOTING;
        setpoint = angleRot;
    }

    public void setIdle() {
        if (state == State.CALIBRATING)
            return;

        if (state != State.IDLE)
            calibrated = false;

        setTargetAngle(NTData.SHOOTER_PIVOT_IDLE_ANGLE.get() / 360.0);
        state = State.IDLE;
    }

    public void setNeutral() {
        if (state == State.CALIBRATING)
            return;

        if (state != State.IDLE)
            calibrated = false;

        motor.setControl(new NeutralOut());
        state = State.IDLE;
    }

    @Override
    public void periodic() {
        if (NTData.SHOOTER_PIVOT_RECALIBRATE.get()) {
            NTData.SHOOTER_PIVOT_RECALIBRATE.set(false);
            state = State.CALIBRATING;
            calibrated = false;
        }

        if (state != State.SHOOTING && !calibrated) {
            motor.setControl(new VoltageOut(-NTData.SHOOTER_PIVOT_CALIBRATE_VOLTS.get()));
            limitSwitch.refresh();
            boolean atLimit = limitSwitch.getValue() == ReverseLimitValue.ClosedToGround;

            if (atLimit) {
                motor.setPosition(hardStopAngle);

                calibrated = true;
                if (state == State.CALIBRATING)
                    state = State.IDLE;
            }
        }
    }

    public boolean hasCalibrated() {
        // Motor knows position after we leave CALIBRATING
        return state != State.CALIBRATING;
    }

    public boolean isAtSetpoint() {
        return state == State.SHOOTING
                && MathUtil.percentError(position.getValue(), setpoint) < NTData.SHOOTER_PIVOT_ALLOWABLE_PCT_ERR.get();
    }

    @Override
    public void simulationPeriodic() {
        motor.updateSim(12);
        position.refresh();
    }

    private void applyPID(Slot0Configs config) {
        config.kP = NTData.SHOOTER_PIVOT_KP.get();
        config.kD = NTData.SHOOTER_PIVOT_KD.get();
        config.kV = NTData.SHOOTER_PIVOT_KV.get();
    }

    private void updatePID(double ignored) {
        Slot0Configs configs = new Slot0Configs();
        applyPID(configs);
        motor.getConfigurator().apply(configs);
    }

    public TalonFXWithSim getMotor() {
        return motor;
    }
}
