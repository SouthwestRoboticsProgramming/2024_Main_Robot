package com.swrobotics.robot.subsystems.tagtracker;

public final class CameraCaptureProperties {
    private boolean autoExposure = false;
    private double exposure = 30;
    private double gain = 1;
    private double targetFps = 50;

    public CameraCaptureProperties setAutoExposure(boolean autoExposure) {
        this.autoExposure = autoExposure;
        return this;
    }

    public CameraCaptureProperties setExposure(double exposure) {
        this.exposure = exposure;
        return this;
    }

    public CameraCaptureProperties setGain(double gain) {
        this.gain = gain;
        return this;
    }

    public CameraCaptureProperties setTargetFps(double targetFps) {
        this.targetFps = targetFps;
        return this;
    }

    public boolean isAutoExposure() {
        return autoExposure;
    }

    public double getExposure() {
        return exposure;
    }

    public double getGain() {
        return gain;
    }

    public double getTargetFps() {
        return targetFps;
    }
}
