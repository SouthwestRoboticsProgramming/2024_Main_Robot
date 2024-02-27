package com.swrobotics.robot.subsystems.speaker;

import com.ctre.phoenix6.hardware.TalonFX;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.subsystems.speaker.aim.AimCalculator;
import com.swrobotics.robot.subsystems.speaker.aim.TableAimCalculator;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ShooterSubsystem extends SubsystemBase {
    public enum FlywheelControl {
        SHOOT,
        POOP,
        IDLE,
        REVERSE
    }

    private static final Pose2d blueSpeakerPose = new Pose2d(0, 5.5475, new Rotation2d(0));

    private final PivotSubsystem pivot;
    private final FlywheelSubsystem flywheel;

    private final SwerveDrive drive;
    private final IndexerSubsystem indexer;

    private Debouncer afterShootDelay;
    private boolean isPreparingToShoot;

    private AimCalculator.Aim targetAim; // Target aim is null if not currently aiming
    private AimCalculator aimCalculator;

    private FlywheelControl flywheelControl;

    public ShooterSubsystem(SwerveDrive drive, IndexerSubsystem indexer) {
        this.pivot = new PivotSubsystem();
        this.flywheel = new FlywheelSubsystem();

        this.drive = drive;
        this.indexer = indexer;

//        aimCalculator = new ManualAimCalculator();
        aimCalculator = TableAimCalculator.INSTANCE;

        NTData.SHOOTER_AFTER_DELAY.nowAndOnChange((delay) -> afterShootDelay = new Debouncer(delay, Debouncer.DebounceType.kFalling));

        flywheelControl = FlywheelControl.SHOOT;
    }

    public Translation2d getSpeakerPosition() {
        return drive.getFieldInfo().flipPoseForAlliance(blueSpeakerPose).getTranslation();
    }

    @Override
    public void periodic() {
        isPreparingToShoot = false;

        double distToSpeaker = getSpeakerPosition().getDistance(drive.getEstimatedPose().getTranslation());
        AimCalculator.Aim aim = aimCalculator.calculateAim(distToSpeaker);
        targetAim = aim;

        // Wait until the pivot is calibrated to do any shooting
        if (DriverStation.isDisabled() || !pivot.hasCalibrated())
            return;

        // To get note unstuck if we squish it
        if (flywheelControl == FlywheelControl.REVERSE) {
            pivot.setNeutral(); // Don't squish note any more than we are currently
            flywheel.setDutyCycle(-NTData.SHOOTER_FLYWHEEL_REVERSE_SPEED.get());
            return;
        }

        if (flywheelControl == FlywheelControl.POOP) {
            pivot.setIdle();
            flywheel.setDutyCycle(NTData.SHOOTER_FLYWHEEL_POOP_SPEED.get());
            return;
        }

        boolean inAuto = DriverStation.isAutonomous();
        boolean hasPiece = indexer.hasPiece();
        boolean shouldContinue = afterShootDelay.calculate(hasPiece);

        if (flywheelControl == FlywheelControl.SHOOT && (shouldContinue || inAuto)) {
            // Actually do shooting
            isPreparingToShoot = true;
            flywheel.setTargetVelocity(aim.flywheelVelocity());
            pivot.setShooting(aim.pivotAngle() / MathUtil.TAU);
        } else {
            // In IDLE and continue time has elapsed
            pivot.setIdle();

            if (hasPiece)
                flywheel.setDutyCycle(NTData.SHOOTER_FLYWHEEL_IDLE_SPEED.get());
            else
                flywheel.setNeutral();
        }

        NTData.SHOOTER_READY.set(isReadyToShoot());
        pctErr.set(flywheel.getPercentErr());

        aimCalculator = TableAimCalculator.INSTANCE;
    }

    NTDouble pctErr = new NTDouble("Shooter/Debug/Percent Error", 0);

    @Override
    public void simulationPeriodic() {
        if (isPreparingToShoot)
            SimView.targetTrajectory.update(targetAim);
        else
            SimView.targetTrajectory.clear();
    }

    public boolean isPreparingToShoot() {
        return isPreparingToShoot;
    }

    public boolean isReadyToShoot() {
        return isPreparingToShoot && flywheel.isReadyToShoot() && pivot.isAtSetpoint();
    }

    public void setFlywheelControl(FlywheelControl flywheelControl) {
        this.flywheelControl = flywheelControl;
    }

    public double getFlywheelPercentOfTarget() {
        return flywheel.getPercentOfTarget();
    }

    public void setTempAimCalculator(AimCalculator calculator) {
        aimCalculator = calculator;
    }

    public AimCalculator.Aim getTargetAim() {
        return targetAim;
    }

    public TalonFX getLeftFlywheelMotor() {
        return flywheel.getLeftMotor();
    }

    public TalonFX getRightFlywheelMotor() {
        return flywheel.getRightMotor();
    }

    public TalonFX getPivotMotor() {
        return pivot.getMotor();
    }
}
