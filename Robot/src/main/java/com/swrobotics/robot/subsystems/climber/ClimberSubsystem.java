package com.swrobotics.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ClimberSubsystem extends SubsystemBase {
    private enum ArmState {
        EXTENDING,
        RETRACTING,
        HOLD
    }

    private static final class Arm {
        private final TalonFX motor;

        public Arm(IOAllocation.CanId motorId) {
            motor = new TalonFX(motorId.id(), motorId.bus());

            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // FIXME
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }

        public void setState(ArmState state) {
            double speed = switch (state) {
                case EXTENDING -> NTData.CLIMBER_EXTEND_SPEED.get();
                case RETRACTING -> -NTData.CLIMBER_RETRACT_SPEED.get();
                case HOLD -> 0;
            };

            // TODO: Do we want to use torque or voltage output for retracting?
            motor.setControl(new DutyCycleOut(speed));
        }
    }

    private final Arm leftArm = new Arm(IOAllocation.CAN.CLIMBER_L_MOTOR);
    private final Arm rightArm = new Arm(IOAllocation.CAN.CLIMBER_R_MOTOR);

    // TODO: We could add auto-balancing retracting one arm more than the other
    public void setState(ArmState state) {
        leftArm.setState(state);
        rightArm.setState(state);
    }
}
