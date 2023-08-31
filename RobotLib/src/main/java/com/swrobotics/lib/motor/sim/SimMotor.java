package com.swrobotics.lib.motor.sim;

import com.swrobotics.lib.motor.Motor;
import com.swrobotics.lib.motor.MotorType;
import com.swrobotics.mathlib.Angle;
import com.swrobotics.mathlib.MathUtil;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SimMotor extends SubsystemBase implements Motor {
    private static final class CurrentControl<D> {
        private final SimControlMode<D> controlMode;
        private final D currentDemand;
        private final double arbFF;

        public CurrentControl(SimControlMode<D> controlMode, D currentDemand, double arbFF) {
            this.controlMode = controlMode;
            this.currentDemand = currentDemand;
            this.arbFF = arbFF;
        }

        public double calc() {
            return controlMode.calc(currentDemand) + arbFF;
        }
    }

    protected final MotorType type;
    private final SimPercentOutCtrl percentOut;

    private CurrentControl<?> currentControl;
    protected Angle rawAngle;
    protected double flip;

    public SimMotor(MotorType type) {
        this.type = type;
        percentOut = new SimPercentOutCtrl();

        currentControl = null;
        rawAngle = Angle.ZERO;
        flip = 1;
    }

    public <D> void apply(SimControlMode<D> mode, D demand) {
        apply(mode, demand, 0);
    }

    public <D> void apply(SimControlMode<D> mode, D demand, double arbFF) {
        if (currentControl == null || currentControl.controlMode != mode) {
            mode.begin();
        }
        currentControl = new CurrentControl<>(mode, demand, arbFF);
    }

    @Override
    public void simulationPeriodic() {
        double pctOut = currentControl == null ? 0 : currentControl.calc();

        // TODO: Real physics
        rawAngle = rawAngle.add(type.freeSpeed.mul(MathUtil.clamp(pctOut, -1, 1) * flip * 0.02));
    }

    @Override
    public void setPercentOut(double percent) {
        apply(percentOut, percent);
    }

    public void setInverted(boolean inverted) {
        flip = inverted ? -1 : 1;
    }
}
