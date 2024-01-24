package com.swrobotics.robot.subsystems.speaker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkLowLevel;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.subsystems.amp.AmpArmSubsystem;
import com.swrobotics.robot.utils.SparkMaxWithSim;
import com.swrobotics.robot.utils.TalonFXWithSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class IntakeSubsystem extends SubsystemBase {
    private static final double motorToIntakeRatio = 2; // FIXME: Probably is not 2:1

    private final CANcoder actuatorEncoder = new CANcoder(IOAllocation.CAN.INTAKE_ACTUATOR_CANCODER.id(), IOAllocation.CAN.INTAKE_ACTUATOR_CANCODER.bus());
//    private final TalonFXWithSim actuatorMotor = new TalonFXWithSim(
//            IOAllocation.CAN.INTAKE_ACTUATOR_MOTOR.id(),
//            IOAllocation.CAN.INTAKE_ACTUATOR_MOTOR.bus(),
//            DCMotor.getFalcon500Foc(1),
//            motorToIntakeRatio,
//            0.005)
//            .attachCanCoder(actuatorEncoder);
    private final SparkMaxWithSim actuatorMotor = SparkMaxWithSim.create(
            IOAllocation.CAN.INTAKE_ACTUATOR_MOTOR,
            CANSparkLowLevel.MotorType.kBrushless,
            DCMotor.getNEO(1),
            motorToIntakeRatio,
            0.005)
            .attachCanCoder(actuatorEncoder);
    private final PWMTalonSRX spinMotor;

    private final StatusSignal<Double> encoderPosition;

    private boolean active;

    public IntakeSubsystem() {
//        TalonFXConfiguration config = new TalonFXConfiguration();
//        config.Slot0.kP = 0.15;
//        config.Slot0.kI = 0;
//        config.Slot0.kD = 1;
//        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // FIXME: Which one?
//        config.Feedback.SensorToMechanismRatio = motorToIntakeRatio;
//        actuatorMotor.getConfigurator().apply(config);

        actuatorMotor.setPID(NTData.INTAKE_KP, NTData.INTAKE_KI, NTData.INTAKE_KD);
        actuatorMotor.setRotorToMechanismRatio(motorToIntakeRatio);
        actuatorMotor.setInverted(false); // FIXME

        encoderPosition = actuatorEncoder.getAbsolutePosition();
        spinMotor = new PWMTalonSRX(IOAllocation.RIO.PWM_INTAKE_MOTOR);

        // Assume intake is retracted (as it should be at the start of a match)
        actuatorMotor.setPosition(getPositionWithCanCoder());
    }

    public void set(boolean active) {
        this.active = active;
        actuatorMotor.setPosition(active ? NTData.INTAKE_RANGE.get() / 360 : 0);
        spinMotor.set(active ? NTData.INTAKE_SPEED.get() : 0);
    }

    public boolean isActive() {
        return active;
    }

    @Override
    public void periodic() {
        if (NTData.INTAKE_CALIBRATE.get()) {
            NTData.INTAKE_CALIBRATE.set(false);
            calibrateCanCoder();
        }
    }

    @Override
    public void simulationPeriodic() {
        actuatorMotor.updateSim(12);
        SimView.updateIntake(actuatorMotor.getEncoderPosition());
    }

    private double getPositionWithCanCoder() {
        encoderPosition.refresh();
        return encoderPosition.getValue() + NTData.INTAKE_CANCODER_OFFSET.get();
    }

    // Assumes intake is fully up
    private void calibrateCanCoder() {
        encoderPosition.refresh();
        double currentPosition = encoderPosition.getValue();
        double expectedPosition = 0;

        NTData.INTAKE_CANCODER_OFFSET.set(expectedPosition - currentPosition);
    }
}
