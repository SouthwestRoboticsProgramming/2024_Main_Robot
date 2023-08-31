package com.swrobotics.lib.motor.sim;

import com.swrobotics.lib.encoder.Encoder;
import com.swrobotics.lib.encoder.SimEncoder;
import com.swrobotics.lib.motor.FeedbackMotor;
import com.swrobotics.lib.motor.MotorType;
import com.swrobotics.mathlib.Angle;

public class SimFeedbackMotor extends SimMotor implements FeedbackMotor {
    protected final SimEncoder encoder;
    protected final SimPIDController pid;
    private final SimPositionPIDCtrl positionCtrl;
    private final SimVelocityPIDCtrl velocityCtrl;

    public SimFeedbackMotor(
            MotorType type,
            double pidCalcInterval,
            double pidOutputScale,
            double controllerSensorUnitsPerRot,
            double controllerSensorUnitsPerRPS) {
        super(type);
        this.encoder = new SimEncoder(() -> rawAngle.mul(flip));

        pid = new SimPIDController(pidCalcInterval, pidOutputScale);
        positionCtrl =
                new SimPositionPIDCtrl(
                        pid, encoder, type.sensorUnitsPerRot * controllerSensorUnitsPerRot);
        velocityCtrl =
                new SimVelocityPIDCtrl(
                        pid, encoder, type.sensorUnitsPerRPS * controllerSensorUnitsPerRPS);
    }

    @Override
    public void setPositionArbFF(Angle position, double arbFF) {
        apply(positionCtrl, position, arbFF);
    }

    @Override
    public void setVelocityArbFF(Angle velocity, double arbFF) {
        apply(velocityCtrl, velocity, arbFF);
    }

    @Override
    public Encoder getIntegratedEncoder() {
        return encoder;
    }

    @Override
    public void resetIntegrator() {
        pid.reset();
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
        pid.setF(kF);
    }
}
