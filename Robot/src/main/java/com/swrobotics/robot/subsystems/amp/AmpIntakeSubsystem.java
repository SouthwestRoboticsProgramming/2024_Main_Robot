package com.swrobotics.robot.subsystems.amp;

import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class AmpIntakeSubsystem extends SubsystemBase {
    public enum State {
        INTAKE,
        OUTTAKE,
        OFF
    }

    private final PWMTalonSRX motor = new PWMTalonSRX(IOAllocation.RIO.PWM_AMP_INTAKE_MOTOR);

    public void setState(State state) {
        double speed = switch (state) {
            case INTAKE -> NTData.AMP_INTAKE_INTAKE_SPEED.get();
            case OUTTAKE -> NTData.AMP_INTAKE_OUTTAKE_SPEED.get();
            case OFF -> 0;
        };

        motor.set(speed);
    }
}
