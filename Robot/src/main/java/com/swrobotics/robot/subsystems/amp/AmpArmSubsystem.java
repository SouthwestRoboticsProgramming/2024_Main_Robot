package com.swrobotics.robot.subsystems.amp;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.utils.CANcoderPositionCalc;
import com.swrobotics.robot.utils.TalonFXWithSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class AmpArmSubsystem extends SubsystemBase {
    public enum Position {
        STOW(NTData.AMP_ARM_STOW_ANGLE),
        PICKUP(NTData.AMP_ARM_PICKUP_ANGLE),
        SCORE_AMP(NTData.AMP_ARM_AMP_SCORE_ANGLE),
        SCORE_TRAP(NTData.AMP_ARM_TRAP_SCORE_ANGLE);

        private final NTEntry<Double> position;

        Position(NTEntry<Double> position) {
            this.position = position;
        }
    }

    private static final double motorToArmRatio = 50;
    private static final double cancoderToArmRatio = 2;

    private final CANcoder absoluteEncoder = new CANcoder(IOAllocation.CAN.AMP_ARM_CANCODER.id(), IOAllocation.CAN.AMP_ARM_CANCODER.bus());
    private final TalonFXWithSim motor = new TalonFXWithSim(
            IOAllocation.CAN.AMP_ARM_MOTOR,
            DCMotor.getFalcon500Foc(1),
            motorToArmRatio,
            0.01)
            .attachCanCoder(absoluteEncoder);
    private final CANcoderPositionCalc positionCalc;

    private Position targetPosition;

    public AmpArmSubsystem() {
        // TODO: Do we want a feedforward to account for gravity?

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kP = NTData.AMP_ARM_KP.get();
        motorConfig.Slot0.kI = 0;
        motorConfig.Slot0.kD = NTData.AMP_ARM_KD.get();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // FIXME
        motorConfig.Feedback.SensorToMechanismRatio = motorToArmRatio;
        motor.getConfigurator().apply(motorConfig);

        NTData.AMP_ARM_KP.onChange(this::updatePID);
        NTData.AMP_ARM_KD.onChange(this::updatePID);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = 0;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // FIXME: Should match motor direction
        absoluteEncoder.getConfigurator().apply(encoderConfig);

        StatusSignal<Double> motorPosition = motor.getPosition();
        positionCalc = new CANcoderPositionCalc(
                absoluteEncoder,
                () -> {
                    motorPosition.refresh();
                    return motorPosition.getValue();
                },
                NTData.AMP_ARM_CANCODER_OFFSET,
                new CANcoderPositionCalc.PositionOverlapResolver(
                        cancoderToArmRatio, 0, 1
                )
        );
        targetPosition = Position.STOW;
    }

    private void updatePID(double ignored) {
        Slot0Configs configs = new Slot0Configs();
        configs.kP = NTData.AMP_ARM_KP.get();
        configs.kI = 0;
        configs.kD = NTData.AMP_ARM_KD.get();
        motor.getConfigurator().apply(configs);
    }

    public void setPosition(Position position) {
    }

    public boolean isAtPosition(Position position) {
        return Math.abs(this.positionCalc.getAbsolute() - getTarget(position)) < NTData.AMP_ARM_TOLERANCE.get() / 360.0;
    }

    private double getTarget(Position position) {
        return position.position.get() / 360.0;
    }

    @Override
    public void periodic() {
        if (NTData.AMP_ARM_CALIBRATE.get()) {
            NTData.AMP_ARM_CALIBRATE.set(false);
            // Assume arm is physically in STOW position
            positionCalc.calibrateCanCoder(getTarget(Position.STOW));
        }

        if (!DriverStation.isDisabled()) {
            double currentPos = this.positionCalc.getAbsolute();
            double targetPos = this.positionCalc.getRelativeForAbsolute(getTarget(targetPosition));

            double gravityFF = NTData.AMP_ARM_GRAVITY_AMOUNT.get() * Math.cos(currentPos);

            motor.setControl(new PositionVoltage(targetPos).withFeedForward(gravityFF));
        }
    }

    @Override
    public void simulationPeriodic() {
        motor.updateSim(12);
        SimView.updateAmpArm(motor.getPosition().getValue());
    }

    public TalonFX getMotor() {
        return motor;
    }
}
