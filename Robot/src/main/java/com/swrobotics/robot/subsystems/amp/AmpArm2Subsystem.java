package com.swrobotics.robot.subsystems.amp;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.lib.net.NTInteger;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.IOAllocation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class AmpArm2Subsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(IOAllocation.CAN.AMP_ARM_MOTOR.id(), IOAllocation.CAN.AMP_ARM_MOTOR.bus());

    public static final NTEntry<Double> kP = new NTDouble("Amp Arm Test/kP", 0);
    public static final NTEntry<Double> kD = new NTDouble("Amp Arm Test/kD", 0);
    public static final NTEntry<Double> target = new NTDouble("Amp Arm Test/Control Demand", 0);
    public static final NTEntry<Double> initialPos = new NTDouble("Amp Arm Test/Initial Position (deg)", 90);

    public static final NTEntry<Double> voltsPerNm = new NTDouble("Amp Arm Test/Volts per N-m", 0);

    public static final NTEntry<Integer> controlType = new NTInteger("Amp Arm Test/Control Mode (0=off,1=volt,2=pos deg)", 0);

    private static final double motorToArmRatio = 50;

    // TODO
    private static final double referenceBaseAngle = 0;
    private static final double referenceIntakeAngle = 0;

    private static final double baseLength = Units.inchesToMeters(19.75);
    private static final double baseRadius = baseLength / 2; // Half of length - distance to center of mass
    private static final double baseMass = 0.25734; // kg

    private static final double intakeRadius = Units.inchesToMeters(7.490789); // Pivot to center of mass
    private static final double intakeMass = 0; // kg

    public AmpArm2Subsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kP.get();
        config.Slot0.kD = kD.get();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = motorToArmRatio;
        motor.getConfigurator().apply(config);

        motor.setPosition(initialPos.get() / 360.0);

        kP.onChange((p) -> updatePD());
        kD.onChange((d) -> updatePD());
    }

    private void updatePD() {
        motor.getConfigurator().apply(new Slot0Configs()
        .withKP(kP.get())
        .withKD(kD.get()));
    }

    @Override
    public void periodic() {
        switch (controlType.get()) {
            case 0:
                motor.setControl(new NeutralOut());
                break;
            case 1:
                motor.setControl(new VoltageOut(target.get()));
                break;
            case 2:
                motor.setControl(new PositionVoltage(target.get() / 360.0));
                break;
        }
    }

    private double calcHoldFeedforward(double baseAngle) {
        double baseGravityForce = MathUtil.G_ACCEL * baseMass * Math.cos(baseAngle);
        double baseTorque = baseRadius * baseGravityForce;

        double intakePivotX = baseLength * Math.cos(baseAngle);
        double intakePivotY = baseLength * Math.sin(baseAngle);

        double intakeAngle = 0; // Angle from horizontal of intake  TODO
        double intakeCgX = intakePivotX + intakeRadius * Math.cos(intakeAngle);
        double intakeCgY = intakePivotY + intakeRadius * Math.sin(intakeAngle);
        double intakeCgDist = Math.hypot(intakeCgX, intakeCgY);

        double intakeGravityForce = MathUtil.G_ACCEL * intakeMass * Math.cos(intakeAngle);
        double intakeTorque = intakeCgDist * intakeGravityForce;

        double netTorque = baseTorque + intakeTorque;
        return netTorque * voltsPerNm.get(); // Probably wrong model of motor but too bad
    }

    public TalonFX getMotor() {
        return motor;
    }
}
