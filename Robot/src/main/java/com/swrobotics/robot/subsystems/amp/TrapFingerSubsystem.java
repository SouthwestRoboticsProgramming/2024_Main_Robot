package com.swrobotics.robot.subsystems.amp;

import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class TrapFingerSubsystem extends SubsystemBase {
    private final Servo servo;

    public TrapFingerSubsystem() {
        servo = new Servo(IOAllocation.RIO.PWM_TRAP_FINGER_SERVO);
        servo.setAngle(NTData.TRAP_FINGER_HOLD_ANGLE.get());
    }

    public void release() {
        servo.setAngle(NTData.TRAP_FINGER_RELEASE_ANGLE.get());
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled())
          servo.setAngle(NTData.TRAP_FINGER_HOLD_ANGLE.get());
    }
}
