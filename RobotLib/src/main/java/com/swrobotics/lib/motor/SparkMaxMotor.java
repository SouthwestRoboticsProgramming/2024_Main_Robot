package com.swrobotics.lib.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.swrobotics.lib.encoder.Encoder;
import com.swrobotics.lib.motor.sim.*;
import com.swrobotics.mathlib.Angle;
import com.swrobotics.mathlib.CCWAngle;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public interface SparkMaxMotor extends FeedbackMotor {
    static SparkMaxMotor brushed(int canId, MotorType type) {
        if (RobotBase.isSimulation()) {
            return new Sim(type);
        }
        return new Real(canId, CANSparkMaxLowLevel.MotorType.kBrushed);
    }

    static SparkMaxMotor neo(int canId, MotorType type) {
        if (RobotBase.isSimulation()) {
            return new Sim(type);
        }
        return new Real(canId, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    // Add special functionality here

    void setPositionSmartMotion(Angle position);

    void setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy accelStrategy);

    void setSmartMotionMinOutputVelocity(Angle minVel);

    void setSmartMotionMaxVelocity(Angle maxVel);

    void setSmartMotionMaxAccel(Angle maxAccel);

    void setSmartMotionAllowedClosedLoopErr(Angle maxErr);

    final class Real implements SparkMaxMotor {
        private final CANSparkMax spark;
        private final SparkMaxPIDController pid;
        private final Encoder encoder;

        private Real(int canId, CANSparkMaxLowLevel.MotorType type) {
            spark = new CANSparkMax(canId, type);
            pid = spark.getPIDController();

            RelativeEncoder encoder = spark.getEncoder();
            encoder.setPositionConversionFactor(1); // Rotations
            encoder.setVelocityConversionFactor(1); // RPM
            pid.setFeedbackDevice(encoder);

            this.encoder =
                    new Encoder() {
                        @Override
                        public Angle getAngle() {
                            return CCWAngle.rot(encoder.getPosition());
                        }

                        @Override
                        public Angle getVelocity() {
                            return CCWAngle.rad(
                                    Units.rotationsPerMinuteToRadiansPerSecond(
                                            encoder.getVelocity()));
                        }

                        @Override
                        public void setAngle(Angle angle) {
                            encoder.setPosition(angle.ccw().rot());
                        }

                        @Override
                        public void setInverted(boolean inverted) {
                            encoder.setInverted(inverted);
                        }
                    };
        }

        @Override
        public void setPercentOut(double pct) {
            pid.setReference(pct, CANSparkMax.ControlType.kDutyCycle);
        }

        @Override
        public void setPositionArbFF(Angle position, double arbFF) {
            pid.setReference(
                    position.ccw().rot(),
                    CANSparkMax.ControlType.kPosition,
                    0,
                    arbFF,
                    SparkMaxPIDController.ArbFFUnits.kPercentOut);
        }

        @Override
        public void setVelocityArbFF(Angle velocity, double arbFF) {
            pid.setReference(
                    Units.radiansPerSecondToRotationsPerMinute(velocity.ccw().rad()),
                    CANSparkMax.ControlType.kVelocity,
                    0,
                    arbFF,
                    SparkMaxPIDController.ArbFFUnits.kPercentOut);
        }

        @Override
        public Encoder getIntegratedEncoder() {
            return encoder;
        }

        @Override
        public void resetIntegrator() {
            pid.setIAccum(0);
        }

        @Override
        public void setInverted(boolean inverted) {
            spark.setInverted(inverted);
        }

        @Override
        public void setP(double kP) {
            pid.setP(kP);
        }

        @Override
        public void setI(double kI) {
            pid.setI(kI);
        }

        @Override
        public void setD(double kD) {
            pid.setD(kD);
        }

        @Override
        public void setF(double kF) {
            pid.setFF(kF);
        }

        @Override
        public void setBrakeMode(boolean brake) {
            spark.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
        }

        @Override
        public void setPositionSmartMotion(Angle position) {
            pid.setReference(position.ccw().rot(), CANSparkMax.ControlType.kSmartMotion);
        }

        @Override
        public void setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy accelStrategy) {
            pid.setSmartMotionAccelStrategy(accelStrategy, 0);
        }

        @Override
        public void setSmartMotionMinOutputVelocity(Angle minVel) {
            pid.setSmartMotionMinOutputVelocity(
                    Units.radiansPerSecondToRotationsPerMinute(minVel.ccw().abs().rad()), 0);
        }

        @Override
        public void setSmartMotionMaxVelocity(Angle maxVel) {
            pid.setSmartMotionMaxVelocity(
                    Units.radiansPerSecondToRotationsPerMinute(maxVel.ccw().abs().rad()), 0);
        }

        @Override
        public void setSmartMotionMaxAccel(Angle maxAccel) {
            pid.setSmartMotionMaxAccel(
                    Units.radiansPerSecondToRotationsPerMinute(maxAccel.ccw().abs().rad()), 0);
        }

        @Override
        public void setSmartMotionAllowedClosedLoopErr(Angle maxErr) {
            pid.setSmartMotionAllowedClosedLoopError(maxErr.ccw().abs().rot(), 0);
        }
    }

    final class Sim extends SimFeedbackMotor implements SparkMaxMotor {
        private final SimSmartMotionCtrl smartMotion;

        private Sim(MotorType type) {
            super(type, 0.001, 1, 1, 60);
            smartMotion = new SimSmartMotionCtrl();
        }

        @Override
        public void setPositionSmartMotion(Angle position) {
            apply(smartMotion, position);
        }

        @Override
        public void setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy accelStrategy) {
            // TODO
        }

        @Override
        public void setSmartMotionMinOutputVelocity(Angle minVel) {
            // TODO
        }

        @Override
        public void setSmartMotionMaxVelocity(Angle maxVel) {
            // TODO
        }

        @Override
        public void setSmartMotionMaxAccel(Angle maxAccel) {
            // TODO
        }

        @Override
        public void setSmartMotionAllowedClosedLoopErr(Angle maxErr) {
            // TODO
        }

        private static final class SimSmartMotionCtrl implements SimControlMode<Angle> {
            @Override
            public void begin() {}

            @Override
            public double calc(Angle demand) {
                // TODO: Figure out how SmartMotion actually works
                return 0;
            }
        }
    }
}
