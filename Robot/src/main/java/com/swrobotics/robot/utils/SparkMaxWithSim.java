package com.swrobotics.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.*;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.IOAllocation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import java.util.function.Supplier;

public interface SparkMaxWithSim {
    static SparkMaxWithSim create(IOAllocation.CanId canId, CANSparkLowLevel.MotorType type, DCMotor motor, double gearRatio, double moi) {
        if (RobotBase.isSimulation()) {
            return new Sim(motor, gearRatio, moi);
        } else {
            return new Real(canId.id(), type);
        }
    }

    void setInverted(boolean inverted);

    void setIdleMode(CANSparkBase.IdleMode mode);

    void setPID(NTEntry<Double> kP, NTEntry<Double> kI, NTEntry<Double> kD);

    void setRotorToMechanismRatio(double ratio);

    void setVoltage(double volts);

    void setPosition(double position);

    void stop();

    // Mechanism position
    double getEncoderPosition();

    void setEncoderPosition(double mechanismPos);


    // Sim specific
    void updateSim(double supplyVolts);
    SparkMaxWithSim attachCanCoder(CANcoder encoder);

    final class Real implements SparkMaxWithSim {
        private final CANSparkMax spark;
        private final SparkPIDController pid;
        private final RelativeEncoder encoder;
        private double rotorToMechanismRatio;

        public Real(int canId, CANSparkLowLevel.MotorType type) {
            spark = new CANSparkMax(canId, type);
            pid = spark.getPIDController();
            encoder = spark.getEncoder();

            rotorToMechanismRatio = 1;
            encoder.setVelocityConversionFactor(1);
        }

        @Override
        public void setInverted(boolean inverted) {
            spark.setInverted(inverted);
        }

        @Override
        public void setIdleMode(CANSparkBase.IdleMode mode) {
            spark.setIdleMode(mode);
        }

        @Override
        public void setPID(NTEntry<Double> kP, NTEntry<Double> kI, NTEntry<Double> kD) {
            kP.nowAndOnChange(pid::setP);
            kI.nowAndOnChange(pid::setI);
            kD.nowAndOnChange(pid::setD);
        }

        @Override
        public void setRotorToMechanismRatio(double ratio) {
            rotorToMechanismRatio = ratio;
        }

        @Override
        public void setVoltage(double volts) {
            pid.setReference(volts, CANSparkBase.ControlType.kVoltage);
        }

        @Override
        public void setPosition(double mechanismPos) {
            pid.setReference(mechanismPos * rotorToMechanismRatio, CANSparkBase.ControlType.kPosition);
        }

        @Override
        public void stop() {
            pid.setReference(0, CANSparkBase.ControlType.kDutyCycle);
        }

        @Override
        public double getEncoderPosition() {
            return encoder.getPosition() / rotorToMechanismRatio;
        }

        @Override
        public void setEncoderPosition(double mechanismPos) {
            encoder.setPosition(mechanismPos * rotorToMechanismRatio);
        }

        @Override
        public void updateSim(double supplyVolts) {
            // Not a sim
        }

        @Override
        public SparkMaxWithSim attachCanCoder(CANcoder encoder) {
            // Not a sim
            return this;
        }
    }

    final class Sim implements SparkMaxWithSim {
        private final DCMotorSim sim;
        private final SimPIDController pid;
        private double rotorToMechanismRatio;
        private double positionOffset;

        private Supplier<Double> controlMode;
        private boolean usingPID;

        private CANcoderSimState absoluteEncoder;

        public Sim(DCMotor motor, double ratio, double moi) {
            sim = new DCMotorSim(motor, ratio, moi);
            pid = new SimPIDController(0.001, 1);
            rotorToMechanismRatio = 1;

            // Control mode returns desired demand in volts
            controlMode = () -> 0.0;
            usingPID = false;

            absoluteEncoder = null;
        }

        @Override
        public void setInverted(boolean inverted) {
            // Sim does not care
        }

        @Override
        public void setIdleMode(CANSparkBase.IdleMode mode) {
            // Sim does not care (yet)
        }

        @Override
        public void setPID(NTEntry<Double> kP, NTEntry<Double> kI, NTEntry<Double> kD) {
            kP.nowAndOnChange(pid::setP);
            kI.nowAndOnChange(pid::setI);
            kD.nowAndOnChange(pid::setD);
        }

        @Override
        public void setRotorToMechanismRatio(double ratio) {
            rotorToMechanismRatio = ratio;
        }

        @Override
        public void setVoltage(double volts) {
            controlMode = () -> volts;
            usingPID = false;
        }

        @Override
        public void setPosition(double position) {
            if (!usingPID)
                pid.reset();
            usingPID = true;

            controlMode = () -> 12 * pid.calc(getRotorPosition(), position * rotorToMechanismRatio);
        }

        @Override
        public void stop() {
            controlMode = () -> 0.0;
            usingPID = false;
        }

        @Override
        public double getEncoderPosition() {
            return getRotorPosition() / rotorToMechanismRatio;
        }

        @Override
        public void setEncoderPosition(double mechanismPos) {
            positionOffset = mechanismPos * rotorToMechanismRatio - sim.getAngularPositionRotations();
        }

        private double getRotorPosition() {
            return sim.getAngularPositionRotations() + positionOffset;
        }

        @Override
        public void updateSim(double supplyVolts) {
            double targetVolts = controlMode.get();
            targetVolts = MathUtil.clamp(targetVolts, -supplyVolts, supplyVolts);

            sim.setInputVoltage(targetVolts);
            sim.update(0.02);

            if (absoluteEncoder != null) {
                absoluteEncoder.setSupplyVoltage(supplyVolts);
                absoluteEncoder.setRawPosition(sim.getAngularPositionRotations() / rotorToMechanismRatio);
                // Velocity is not simulated so too bad
                // No one uses CanCoder for velocity anyway
            }
        }

        @Override
        public SparkMaxWithSim attachCanCoder(CANcoder encoder) {
            absoluteEncoder = encoder.getSimState();
            return this;
        }
    }
}
