package com.swrobotics.robot.subsystems.amp;

import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class AmpIntakeSubsystem extends SubsystemBase {
    public enum State {
        INTAKE,
        OUTTAKE,
        OFF
    }

    private final PWMTalonSRX motor = new PWMTalonSRX(-1);
    private final Timer backoutTimer = new Timer();

    public void setState(State state) {
        double speed = switch (state) {
            case INTAKE -> NTData.AMP_INTAKE_INTAKE_SPEED.get();
            case OUTTAKE -> NTData.AMP_INTAKE_OUTTAKE_SPEED.get();
            case OFF -> 0;
        };

        if (state == State.INTAKE)
            backoutTimer.restart();
        else if (state == State.OFF && !backoutTimer.hasElapsed(NTData.AMP_INTAKE_BACKOUT_TIME.get()))
            speed = -NTData.AMP_INTAKE_BACKOUT_SPEED.get();

        motor.set(speed);
    }
}
