package com.swrobotics.robot.subsystems.speaker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import com.swrobotics.robot.utils.SparkMaxWithSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ShooterSubsystem extends SubsystemBase {
    private static final Pose2d blueSpeakerPose = new Pose2d(0, 5.5475, new Rotation2d(0));
    private static final double motorToPivotRatio = 10; // FIXME
    private static final double hardStopAngle = 30 / 360.0; // FIXME

    private static final record Aim(double flywheelVelocity, double pivotAngle) {}

    private final SwerveDrive drive;
    private final IndexerSubsystem indexer;

    private final TalonFX flywheelMotor1 = new TalonFX(IOAllocation.CAN.SHOOTER_MOTOR_1.id(), IOAllocation.CAN.SHOOTER_MOTOR_1.bus());
    private final TalonFX flywheelMotor2 = new TalonFX(IOAllocation.CAN.SHOOTER_MOTOR_2.id(), IOAllocation.CAN.SHOOTER_MOTOR_2.bus());
//    private final TalonFX pivotMotor = new TalonFX(IOAllocation.CAN.SHOOTER_PIVOT_MOTOR.id(), IOAllocation.CAN.SHOOTER_PIVOT_MOTOR.bus());
    private final CANcoder pivotEncoder = new CANcoder(IOAllocation.CAN.SHOOTER_PIVOT_CANCODER.id(), IOAllocation.CAN.SHOOTER_PIVOT_CANCODER.bus());
    private final SparkMaxWithSim pivotMotor = SparkMaxWithSim.create(
            IOAllocation.CAN.SHOOTER_PIVOT_MOTOR,
            CANSparkLowLevel.MotorType.kBrushless,
            DCMotor.getNEO(1),
            motorToPivotRatio,
            0.01
    ).attachCanCoder(pivotEncoder);

    private final StatusSignal<Double> encoderPosition;

    public ShooterSubsystem(SwerveDrive drive, IndexerSubsystem indexer) {
        this.drive = drive;
        this.indexer = indexer;

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        // TODO: Configure for good velocity feedback/feedforward
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        flywheelMotor1.getConfigurator().apply(flywheelConfig);
        flywheelMotor2.getConfigurator().apply(flywheelConfig);
        flywheelMotor2.setControl(new Follower(IOAllocation.CAN.SHOOTER_MOTOR_1.id(), true)); // FIXME: Is oppose correct here?

//        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
//        pivotConfig.Slot0.kP = 0;
//        pivotConfig.Slot0.kI = 0;
//        pivotConfig.Slot0.kD = 0;
//        pivotConfig.Feedback.SensorToMechanismRatio = motorToPivotRatio;
//        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setPID(NTData.SHOOTER_PIVOT_KP, NTData.SHOOTER_PIVOT_KI, NTData.SHOOTER_PIVOT_KD);
        pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotMotor.setInverted(false); // FIXME
        pivotMotor.setRotorToMechanismRatio(motorToPivotRatio);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // FIXME
        pivotEncoder.getConfigurator().apply(encoderConfig);

        encoderPosition = pivotEncoder.getAbsolutePosition();
        pivotMotor.setPosition(getPivotPositionWithCanCoder());
    }

    // TODO: Maybe account for velocity to shoot on the move
    private Aim calculateAim(double distToSpeaker) {
        // TODO
        return new Aim(10, Math.PI / 4);
    }

    public Translation2d getSpeakerPosition() {
        return drive.getFieldInfo().flipPoseForAlliance(blueSpeakerPose).getTranslation();
    }

    private void setPivotTarget(double angleRot) {
        angleRot = Math.max(angleRot, hardStopAngle);
//        pivotMotor.setControl(new PositionDutyCycle(angleRot));
        pivotMotor.setPosition(angleRot);
    }

    private double getPivotPositionWithCanCoder() {
        encoderPosition.refresh();
        return encoderPosition.getValue() + NTData.SHOOTER_PIVOT_CANCODER_OFFSET.get();
    }

    // Assumes arm is physically at hard-stop position
    private void calibrateCanCoder() {
        encoderPosition.refresh();
        double currentPosition = encoderPosition.getValue();
        NTData.SHOOTER_PIVOT_CANCODER_OFFSET.set(hardStopAngle - currentPosition);
    }

    @Override
    public void periodic() {
        if (NTData.SHOOTER_PIVOT_CALIBRATE.get()) {
            NTData.SHOOTER_PIVOT_CALIBRATE.set(false);
            calibrateCanCoder();
        }

        if (indexer.hasPiece()) {
            double distToSpeaker = getSpeakerPosition().getDistance(drive.getEstimatedPose().getTranslation());

            if (distToSpeaker < NTData.SHOOTER_FULL_SPEED_DISTANCE.get()) {
                Aim aim = calculateAim(distToSpeaker);

                // TODO: Torque/FOC?
                flywheelMotor1.setControl(new VelocityDutyCycle(aim.flywheelVelocity));
                setPivotTarget(aim.pivotAngle / MathUtil.TAU);
            } else {
                flywheelMotor1.setControl(new DutyCycleOut(NTData.SHOOTER_IDLE_SPEED.get()));
                setPivotTarget(NTData.SHOOTER_PIVOT_IDLE_ANGLE.get() / 360.0);
            }
        } else {
            flywheelMotor1.setControl(new NeutralOut());
//            pivotMotor.setControl(new NeutralOut());
            pivotMotor.stop();
        }
    }

    @Override
    public void simulationPeriodic() {
        pivotMotor.updateSim(12);
        SimView.updateShooter(pivotMotor.getEncoderPosition());
    }
}
