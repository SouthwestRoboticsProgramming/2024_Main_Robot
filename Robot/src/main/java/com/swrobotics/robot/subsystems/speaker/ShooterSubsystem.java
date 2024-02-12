package com.swrobotics.robot.subsystems.speaker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import com.swrobotics.robot.utils.CANcoderPositionCalc;
import com.swrobotics.robot.utils.SparkMaxWithSim;
import com.swrobotics.robot.utils.TalonFXWithSim;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ShooterSubsystem extends SubsystemBase {
    private static final Pose2d blueSpeakerPose = new Pose2d(0, 5.5475, new Rotation2d(0));
    private static final double motorToPivotRatio = 10; // FIXME
    private static final double hardStopAngle = 30 / 360.0; // FIXME

    private static final record Aim(double flywheelVelocity, double pivotAngle) {}

    private final SwerveDrive drive;
    private final IndexerSubsystem indexer;

    private static TalonFXWithSim createFlywheel(IOAllocation.CanId id) {
        return new TalonFXWithSim(
                id,
                DCMotor.getFalcon500Foc(1),
                1,
                0.001
        );
    }

    public final TalonFXWithSim flywheelMotor1 = createFlywheel(IOAllocation.CAN.SHOOTER_MOTOR_1);
    public final TalonFXWithSim flywheelMotor2 = createFlywheel(IOAllocation.CAN.SHOOTER_MOTOR_2);
    private final StatusSignal<Double> flywheelVelocity1, flywheelVelocity2;

    private final CANcoder pivotEncoder = new CANcoder(
            IOAllocation.CAN.SHOOTER_PIVOT_CANCODER.id(),
            IOAllocation.CAN.SHOOTER_PIVOT_CANCODER.bus());
    private final SparkMaxWithSim pivotMotor = SparkMaxWithSim.create(
            IOAllocation.CAN.SHOOTER_PIVOT_MOTOR,
            CANSparkLowLevel.MotorType.kBrushless,
            DCMotor.getNEO(1),
            motorToPivotRatio,
            0.01
    ).attachCanCoder(pivotEncoder);
    private final CANcoderPositionCalc pivotPosition;

    private Debouncer afterShootDelay;
    private boolean isPreparing;
    private double targetVelocity;

    public ShooterSubsystem(SwerveDrive drive, IndexerSubsystem indexer) {
        this.drive = drive;
        this.indexer = indexer;

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        applyPIDV(flywheelConfig.Slot0);
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelMotor1.getConfigurator().apply(flywheelConfig);
        flywheelMotor2.getConfigurator().apply(flywheelConfig);
        flywheelVelocity1 = flywheelMotor1.getVelocity();
        flywheelVelocity2 = flywheelMotor2.getVelocity();

        pivotMotor.setPID(NTData.SHOOTER_PIVOT_KP, NTData.SHOOTER_PIVOT_KI, NTData.SHOOTER_PIVOT_KD);
        pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotMotor.setInverted(false); // FIXME
        pivotMotor.setRotorToMechanismRatio(motorToPivotRatio);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // FIXME
        pivotEncoder.getConfigurator().apply(encoderConfig);
        pivotPosition = new CANcoderPositionCalc(pivotEncoder, pivotMotor::getEncoderPosition, NTData.SHOOTER_PIVOT_CANCODER_OFFSET);

        NTData.SHOOTER_AFTER_DELAY.nowAndOnChange((delay) -> afterShootDelay = new Debouncer(delay, Debouncer.DebounceType.kFalling));
        NTData.SHOOTER_FLYWHEEL_KP.onChange(this::updatePIDV);
        NTData.SHOOTER_FLYWHEEL_KI.onChange(this::updatePIDV);
        NTData.SHOOTER_FLYWHEEL_KD.onChange(this::updatePIDV);
        NTData.SHOOTER_FLYWHEEL_KV.onChange(this::updatePIDV);
    }

    private void applyPIDV(Slot0Configs config) {
        config.kP = NTData.SHOOTER_FLYWHEEL_KP.get();
        config.kI = NTData.SHOOTER_FLYWHEEL_KP.get();
        config.kD = NTData.SHOOTER_FLYWHEEL_KP.get();
        config.kV = NTData.SHOOTER_FLYWHEEL_KP.get();
    }

    private void updatePIDV(double ignored) {
        Slot0Configs configs = new Slot0Configs();
        applyPIDV(configs);
        flywheelMotor1.getConfigurator().apply(configs);
        flywheelMotor2.getConfigurator().apply(configs);
    }

    // TODO: Maybe account for velocity to shoot on the move
    private Aim calculateAim(double distToSpeaker) {
        // TODO
        return new Aim(4000/60.0, Math.PI / 4);
    }

    public Translation2d getSpeakerPosition() {
        return drive.getFieldInfo().flipPoseForAlliance(blueSpeakerPose).getTranslation();
    }

    private void setPivotTarget(double angleRot) {
        angleRot = Math.max(angleRot, hardStopAngle);
        pivotMotor.setPosition(pivotPosition.getRelativeForAbsolute(angleRot));
    }

    private void setFlywheelTarget(double velocityRPS) {
        // TODO: Torque/FOC?

        targetVelocity = velocityRPS;
        flywheelMotor1.setControl(new VelocityVoltage(velocityRPS));
        flywheelMotor2.setControl(new VelocityVoltage(velocityRPS));
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled())
            return;

        StatusSignal.refreshAll(flywheelVelocity1, flywheelVelocity2);

        if (NTData.SHOOTER_PIVOT_CALIBRATE.get()) {
            NTData.SHOOTER_PIVOT_CALIBRATE.set(false);
            pivotPosition.calibrateCanCoder(hardStopAngle);
        }

        isPreparing = false;
        if (afterShootDelay.calculate(indexer.hasPiece())) {
            double distToSpeaker = getSpeakerPosition().getDistance(drive.getEstimatedPose().getTranslation());

            if (distToSpeaker < NTData.SHOOTER_FULL_SPEED_DISTANCE.get()) {
                Aim aim = calculateAim(distToSpeaker);

                isPreparing = true;
                setFlywheelTarget(aim.flywheelVelocity);
                setPivotTarget(aim.pivotAngle / MathUtil.TAU);
            } else {
                DutyCycleOut cmd = new DutyCycleOut(NTData.SHOOTER_IDLE_SPEED.get());
                flywheelMotor1.setControl(cmd);
                flywheelMotor2.setControl(cmd);
                setPivotTarget(NTData.SHOOTER_PIVOT_IDLE_ANGLE.get() / 360.0);
            }
        } else {
            flywheelMotor1.setControl(new NeutralOut());
            flywheelMotor2.setControl(new NeutralOut());
            pivotMotor.stop();
        }
    }

    @Override
    public void simulationPeriodic() {
        flywheelMotor1.updateSim(12);
        flywheelMotor2.updateSim(12);
        pivotMotor.updateSim(12);
        SimView.updateShooter(pivotMotor.getEncoderPosition());
    }

    public boolean isPreparing() {
        return isPreparing;
    }

    // For the status indicator in lights
    // <1 if too slow, >1 if too high
    public double getPercentOfTarget() {
        double min = Math.min(flywheelVelocity1.getValue(), flywheelVelocity2.getValue());
        return min / targetVelocity;
    }

    private double getSignedPercentErr() {
        double err1 = MathUtil.signedPercentError(flywheelVelocity1.getValue(), targetVelocity);
        double err2 = MathUtil.signedPercentError(flywheelVelocity2.getValue(), targetVelocity);
        return Math.min(err1, err2);
    }

    public boolean isReadyToShoot() {
        return Math.abs(getSignedPercentErr()) < NTData.SHOOTER_ALLOWABLE_PCT_ERR.get();
    }
}
