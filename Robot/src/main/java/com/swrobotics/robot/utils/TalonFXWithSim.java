package com.swrobotics.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public final class TalonFXWithSim extends TalonFX {
    private final DCMotorSim physicsSim;
    private final TalonFXSimState simState;
    private final double gearRatio;

    private CANcoderSimState absoluteEncoder;

    public TalonFXWithSim(int deviceId, DCMotor motor, double gearRatio, double moi) {
        this(deviceId, "", motor, gearRatio, moi);
    }

    public TalonFXWithSim(int deviceId, String canbus, DCMotor motor, double gearRatio, double moi) {
        super(deviceId, canbus);
        this.gearRatio = gearRatio;

        if (RobotBase.isSimulation()) {
            physicsSim = new DCMotorSim(motor, gearRatio, moi);
            simState = getSimState();
        } else {
            physicsSim = null;
            simState = null;
        }

        absoluteEncoder = null;
    }

    public void updateSim(double supplyVolts) {
        if (physicsSim == null) {
            DriverStation.reportError("Calling updateSim() on real robot!", true);
            return;
        }

        physicsSim.setInputVoltage(simState.getMotorVoltage());
        physicsSim.update(0.02);

        // Relative to mechanism
        double position = physicsSim.getAngularPositionRotations();
        double velocity = physicsSim.getAngularVelocityRPM() / 60.0;

        // Scale by gear ratio to get rotor
        simState.setSupplyVoltage(supplyVolts);
        simState.setRawRotorPosition(position * gearRatio);
        simState.setRotorVelocity(velocity / 60.0 * gearRatio);

        if (absoluteEncoder != null) {
            absoluteEncoder.setSupplyVoltage(supplyVolts);
            absoluteEncoder.setRawPosition(position);
            absoluteEncoder.setVelocity(velocity);
        }
    }

    public TalonFXWithSim attachCanCoder(CANcoder cancoder) {
        absoluteEncoder = cancoder.getSimState();
        return this;
    }
}
