package com.swrobotics.lib.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.swrobotics.lib.encoder.Encoder;
import com.swrobotics.lib.motor.sim.SimFeedbackMotor;
import com.swrobotics.mathlib.Angle;
import com.swrobotics.mathlib.CCWAngle;

import edu.wpi.first.wpilibj.RobotBase;

public interface TalonMotor extends FeedbackMotor {
    static TalonMotor talonFX(int canId) {
        return talonFX(canId, "");
    }

    static TalonMotor talonFX(int canId, String canBus) {
        if (RobotBase.isSimulation()) {
            return new Sim(MotorType.FALCON_500);
        }

        TalonFXConfiguration config = new TalonFXConfiguration();

        // Set initial PIDF to zero so the motor won't move by default
        config.slot0.kP = 0;
        config.slot0.kI = 0;
        config.slot0.kD = 0;
        config.slot0.kF = 0;

        // Select the motor's integrated encoder
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

        TalonFX fx = new TalonFX(canId, canBus);
        fx.configAllSettings(config);
        return new Real(fx, 2048.0);
    }

    class Real implements TalonMotor {
        private final BaseTalon talon;
        private final double encoderTicksPerRotation;
        private final Encoder integratedEncoder;

        private Real(BaseTalon talon, double encoderTicksPerRotation) {
            this.talon = talon;
            this.encoderTicksPerRotation = encoderTicksPerRotation;

            integratedEncoder =
                    new Encoder() {
                        @Override
                        public Angle getAngle() {
                            return CCWAngle.rot(
                                    talon.getSelectedSensorPosition() / encoderTicksPerRotation);
                        }

                        @Override
                        public Angle getVelocity() {
                            return CCWAngle.rot(
                                    talon.getSelectedSensorVelocity()
                                            / encoderTicksPerRotation
                                            * 10);
                        }

                        @Override
                        public void setAngle(Angle angle) {
                            talon.setSelectedSensorPosition(
                                    angle.ccw().rot() * encoderTicksPerRotation);
                        }
                    };
        }

        @Override
        public void setPositionArbFF(Angle position, double arbFF) {
            talon.set(
                    ControlMode.Position,
                    position.ccw().rot() * encoderTicksPerRotation,
                    DemandType.ArbitraryFeedForward,
                    arbFF);
        }

        @Override
        public void setVelocityArbFF(Angle velocity, double arbFF) {
            talon.set(
                    ControlMode.Velocity,
                    velocity.ccw().rot() * encoderTicksPerRotation / 10,
                    DemandType.ArbitraryFeedForward,
                    arbFF);
        }

        @Override
        public Encoder getIntegratedEncoder() {
            return integratedEncoder;
        }

        @Override
        public void resetIntegrator() {
            talon.setIntegralAccumulator(0);
        }

        @Override
        public void setP(double kP) {
            talon.config_kP(0, kP);
        }

        @Override
        public void setI(double kI) {
            talon.config_kI(0, kI);
        }

        @Override
        public void setD(double kD) {
            talon.config_kD(0, kD);
        }

        @Override
        public void setF(double kF) {
            talon.config_kF(0, kF);
        }

        @Override
        public void setPercentOut(double percent) {
            talon.set(ControlMode.PercentOutput, percent);
        }

        @Override
        public void setInverted(boolean inverted) {
            talon.setInverted(inverted);
        }

        @Override
        public void setBrakeMode(boolean brake) {
            talon.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
        }
    }

    class Sim extends SimFeedbackMotor implements TalonMotor {
        private Sim(MotorType type) {
            super(type, 0.001, 1023, 1, 0.1);
        }
    }
}
