package com.swrobotics.robot.subsystems.speaker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.utils.TalonFXWithSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class FlywheelSubsystem extends SubsystemBase {
    private static TalonFXWithSim createFlywheel(IOAllocation.CanId id) {
        return new TalonFXWithSim(
                id,
                DCMotor.getFalcon500Foc(1),
                1,
                0.001
        );
    }

    public final TalonFXWithSim leftMotor = createFlywheel(IOAllocation.CAN.FLYWHEEL_LEFT_MOTOR);
    public final TalonFXWithSim rightMotor = createFlywheel(IOAllocation.CAN.FLYWHEEL_RIGHT_MOTOR);
    private final StatusSignal<Double> leftVelocity, rightVelocity;

    private double targetVelocity;

    public FlywheelSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        applyPIDV(config.Slot0);
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotor.getConfigurator().apply(config);
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(config);

        leftVelocity = leftMotor.getVelocity();
        rightVelocity = rightMotor.getVelocity();

        NTData.SHOOTER_FLYWHEEL_KP.onChange(this::updatePIDV);
        NTData.SHOOTER_FLYWHEEL_KD.onChange(this::updatePIDV);
        NTData.SHOOTER_FLYWHEEL_KV.onChange(this::updatePIDV);
    }

    public void setTargetVelocity(double velocityRPS) {
        targetVelocity = velocityRPS;
        leftMotor.setControl(new VelocityVoltage(velocityRPS));
        rightMotor.setControl(new VelocityVoltage(velocityRPS));
    }

    public void setDutyCycle(double percentOut) {
        DutyCycleOut cmd = new DutyCycleOut(percentOut);
        leftMotor.setControl(cmd);
        rightMotor.setControl(cmd);
    }

    public void setNeutral() {
        leftMotor.setControl(new NeutralOut());
        rightMotor.setControl(new NeutralOut());
    }

    public boolean isReadyToShoot() {
        return RobotBase.isSimulation() || getPercentErr() < NTData.SHOOTER_FLYWHEEL_ALLOWABLE_PCT_ERR.get();
    }

    @Override
    public void periodic() {
        StatusSignal.refreshAll(leftVelocity, rightVelocity);
    }

    @Override
    public void simulationPeriodic() {
        leftMotor.updateSim(12);
        rightMotor.updateSim(12);
    }

    // For the status indicator in lights
    // <1 if too slow, >1 if too high
    public double getPercentOfTarget() {
        double min = Math.min(leftVelocity.getValue(), rightVelocity.getValue());
        return min / targetVelocity;
    }

    public double getPercentErr() {
        double err1 = Math.abs(MathUtil.signedPercentError(leftVelocity.getValue(), targetVelocity));
        double err2 = Math.abs(MathUtil.signedPercentError(rightVelocity.getValue(), targetVelocity));
        return Math.min(err1, err2);
    }

    private void applyPIDV(Slot0Configs config) {
        config.kP = NTData.SHOOTER_FLYWHEEL_KP.get();
        config.kD = NTData.SHOOTER_FLYWHEEL_KD.get();
        config.kV = NTData.SHOOTER_FLYWHEEL_KV.get();
    }

    private void updatePIDV(double ignored) {
        Slot0Configs configs = new Slot0Configs();
        applyPIDV(configs);
        leftMotor.getConfigurator().apply(configs);
        rightMotor.getConfigurator().apply(configs);
    }

    public TalonFXWithSim getLeftMotor() {
        return leftMotor;
    }

    public TalonFXWithSim getRightMotor() {
        return rightMotor;
    }
}
