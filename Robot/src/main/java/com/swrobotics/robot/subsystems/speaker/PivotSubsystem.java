package com.swrobotics.robot.subsystems.speaker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.utils.TalonFXWithSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class PivotSubsystem extends SubsystemBase {
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

    private final CalibrationStrategy calibration;
    private boolean hasCalibrated;

    public PivotSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        applyPID(config.Slot0);
        config.Feedback.SensorToMechanismRatio = motorToPivotRatio;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motor.getConfigurator().apply(config);
        position = motor.getPosition();

        NTData.SHOOTER_PIVOT_KP.onChange(this::updatePID);
        NTData.SHOOTER_PIVOT_KD.onChange(this::updatePID);
        NTData.SHOOTER_PIVOT_KV.onChange(this::updatePID);

        hasCalibrated = RobotBase.isSimulation();

        calibration = new RunUntilLimit();
        calibration.reset();
    }

    public void setTargetAngle(double angleRot) {
        angleRot = MathUtil.clamp(
                angleRot,
                hardStopAngle + 2 / 360.0,
                maxAngle - 2 / 360.0);

        motor.setControl(new PositionVoltage(angleRot));
    }

    public void setIdle() {
        setTargetAngle(NTData.SHOOTER_PIVOT_IDLE_ANGLE.get() / 360.0);
    }

    public void setNeutral() {
        motor.setControl(new NeutralOut());
    }

    @Override
    public void periodic() {
        if (NTData.SHOOTER_PIVOT_RECALIBRATE.get()) {
            NTData.SHOOTER_PIVOT_RECALIBRATE.set(false);
            hasCalibrated = false;
            calibration.reset();
        }

        if (!hasCalibrated && calibration.calibrate()) {
            hasCalibrated = true;
        }
    }

    public boolean hasCalibrated() {
        return hasCalibrated;
    }

    @Override
    public void simulationPeriodic() {
        motor.updateSim(12);
        position.refresh();
        SimView.updateShooterPivot(position.getValue());
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

    private interface CalibrationStrategy {
        void reset();

        // Returns true if calibration finished
        boolean calibrate();
    }

    private final class AssumeAtHardStop implements CalibrationStrategy {
        @Override
        public void reset() {
        }

        @Override
        public boolean calibrate() {
            motor.setPosition(hardStopAngle);
            return true;
        }
    }

    private final class RunUntilLimit implements CalibrationStrategy {
        private final StatusSignal<ReverseLimitValue> limitSwitch = motor.getReverseLimit();

        @Override
        public void reset() {
            // Don't crunch the shooter
            CurrentLimitsConfigs config = new CurrentLimitsConfigs();
            config.StatorCurrentLimitEnable = true;
            config.StatorCurrentLimit = 5; // Amps
            motor.getConfigurator().apply(config);
        }

        @Override
        public boolean calibrate() {
            motor.setControl(new VoltageOut(-NTData.SHOOTER_PIVOT_CALIBRATE_VOLTS.get()));
            limitSwitch.refresh();
            boolean atLimit = limitSwitch.getValue() == ReverseLimitValue.ClosedToGround;

            if (atLimit) {
                // Disable the low current limit
                CurrentLimitsConfigs config = new CurrentLimitsConfigs();
                config.StatorCurrentLimitEnable = false;
                motor.getConfigurator().apply(config);

                motor.setPosition(hardStopAngle);

                return true;
            }

            return false;
        }
    }
}
