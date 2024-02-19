package com.swrobotics.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ClimberSubsystem extends SubsystemBase {
    private final ClimberArm leftArm = new ClimberArm(IOAllocation.CAN.CLIMBER_L_MOTOR, InvertedValue.Clockwise_Positive);
    private final ClimberArm rightArm = new ClimberArm(IOAllocation.CAN.CLIMBER_R_MOTOR, InvertedValue.CounterClockwise_Positive);

    private ClimberArm.State currentState;

    public void setState(ClimberArm.State state) {
        leftArm.setState(state);
        rightArm.setState(state);
        currentState = state;
    }

    @Override
    public void periodic() {
        if (NTData.CLIMBER_RECALIBRATE.get()) {
            NTData.CLIMBER_RECALIBRATE.set(false);

            leftArm.recalibrate();
            rightArm.recalibrate();
        }
    }

    public TalonFX getLeftMotor() {
        return leftArm.getMotor();
    }

    public TalonFX getRightMotor() {
        return rightArm.getMotor();
    }

    public ClimberArm.State getCurrentState() {
        return currentState;
    }
}
