package com.swrobotics.robot.subsystems.climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ClimberArm extends SubsystemBase {
    public enum State {
        RETRACTED_IDLE,
        EXTENDED,
        RETRACTED_HOLD
    }

    private final TalonFX motor;
    private final StatusSignal<Double> motorVelocity;

    private boolean hasCalibrated;
    private Debouncer calibrationDebounce;
    private State targetState;

    private final StatusSignal<Double> motorPositionDebug;

    public ClimberArm(IOAllocation.CanId id, InvertedValue invert) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = invert;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = NTData.CLIMBER_KP.get();
        config.Slot0.kI = 0;
        config.Slot0.kD = NTData.CLIMBER_KD.get();

        motor = new TalonFX(id.id(), id.bus());
        motor.getConfigurator().apply(config);
        motorVelocity = motor.getVelocity();

        NTData.CLIMBER_KP.onChange(this::applyPID);
        NTData.CLIMBER_KD.onChange(this::applyPID);

        hasCalibrated = RobotBase.isSimulation();
        calibrationDebounce = null;
        targetState = State.RETRACTED_IDLE;

        motorPositionDebug = motor.getPosition();
        positionDebug = new NTDouble("Climber/Debug Position (" + id.id() + ")", 0);
    }

    public void setState(State state) {
        targetState = state;
        if (!hasCalibrated)
            return;

        double position = state == State.EXTENDED
                ? NTData.CLIMBER_EXTEND_POSITION.get()
                : 0;

        double feedforward = state == State.RETRACTED_HOLD
                ? -NTData.CLIMBER_HOLD_VOLTS.get()
                : 0;

//        motor.setControl(new PositionVoltage(position)
//                .withFeedForward(feedforward));
    }

    NTDouble positionDebug;

    @Override
    public void periodic() {
        motorPositionDebug.refresh();
        positionDebug.set(motorPositionDebug.getValue());

        if (DriverStation.isDisabled())
            return;

        if (!hasCalibrated) {
            if (calibrationDebounce == null) {
                calibrationDebounce = new Debouncer(
                        NTData.CLIMBER_CALIBRATE_TIME.get(),
                        Debouncer.DebounceType.kBoth
                );
            }

            motorVelocity.refresh();
            double velocity = motorVelocity.getValue();
            boolean reachedHardStop = Math.abs(velocity) < NTData.CLIMBER_CALIBRATE_THRESHOLD.get();

            if (calibrationDebounce.calculate(reachedHardStop)) {
                motor.setPosition(NTData.CLIMBER_CALIBRATE_POSITION.get());
                hasCalibrated = true;
                NTData.CLIMBER_CALIBRATING.set(false);

                setState(targetState);
            } else {
                NTData.CLIMBER_CALIBRATING.set(true);

//                motor.setControl(new VoltageOut(-NTData.CLIMBER_CALIBRATE_VOLTS.get()));
            }
        }
    }

    public void recalibrate() {
        hasCalibrated = false;
        calibrationDebounce = null;
    }

    private void applyPID(double ignored) {
        Slot0Configs config = new Slot0Configs();
        config.kP = NTData.CLIMBER_KP.get();
        config.kI = 0;
        config.kD = NTData.CLIMBER_KD.get();
        motor.getConfigurator().apply(config);
    }

    public TalonFX getMotor() {
        return motor;
    }
}
