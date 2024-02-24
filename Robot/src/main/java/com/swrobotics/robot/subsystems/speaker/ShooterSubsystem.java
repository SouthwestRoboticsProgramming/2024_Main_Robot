package com.swrobotics.robot.subsystems.speaker;

import com.ctre.phoenix6.hardware.TalonFX;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.subsystems.speaker.aim.AimCalculator;
import com.swrobotics.robot.subsystems.speaker.aim.ManualAimCalculator;
import com.swrobotics.robot.subsystems.speaker.aim.TableAimCalculator;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ShooterSubsystem extends SubsystemBase {
    private static final Pose2d blueSpeakerPose = new Pose2d(0, 5.5475, new Rotation2d(0));

    private final PivotSubsystem pivot;
    private final FlywheelSubsystem flywheel;

    private final SwerveDrive drive;
    private final IndexerSubsystem indexer;

    private Debouncer afterShootDelay;
    private boolean isPreparing;

    private AimCalculator.Aim targetAim; // Target aim is null if not currently aiming
    private final AimCalculator aimCalculator;

    private boolean shouldRunFlywheel;

    public ShooterSubsystem(SwerveDrive drive, IndexerSubsystem indexer) {
        this.pivot = new PivotSubsystem();
        this.flywheel = new FlywheelSubsystem();

        this.drive = drive;
        this.indexer = indexer;

//        aimCalculator = new ManualAimCalculator();
        aimCalculator = new TableAimCalculator();

        NTData.SHOOTER_AFTER_DELAY.nowAndOnChange((delay) -> afterShootDelay = new Debouncer(delay, Debouncer.DebounceType.kFalling));

        shouldRunFlywheel = true;
    }

    public Translation2d getSpeakerPosition() {
        return drive.getFieldInfo().flipPoseForAlliance(blueSpeakerPose).getTranslation();
    }

    @Override
    public void periodic() {
        double distToSpeaker = getSpeakerPosition().getDistance(drive.getEstimatedPose().getTranslation());
        AimCalculator.Aim aim = aimCalculator.calculateAim(distToSpeaker);
        targetAim = aim;

        if (DriverStation.isDisabled() || !pivot.hasCalibrated())
            return;

        isPreparing = false;
        if (afterShootDelay.calculate(indexer.hasPiece())) {
            if (aim != null) {
                isPreparing = true;
                if (shouldRunFlywheel)
                    flywheel.setTargetVelocity(aim.flywheelVelocity());
                else
                    flywheel.setIdle();
                pivot.setTargetAngle(aim.pivotAngle() / MathUtil.TAU);
            } else {
                flywheel.setIdle();
                pivot.setIdle();
            }
        } else {
            flywheel.setNeutral();
            pivot.setNeutral();
        }

        NTData.SHOOTER_READY.set(isReadyToShoot());
        pctErr.set(flywheel.getPercentErr());
    }

    NTDouble pctErr = new NTDouble("Shooter/Debug/Percent Error", 0);

    @Override
    public void simulationPeriodic() {
        if (isPreparing)
            SimView.targetTrajectory.update(targetAim);
        else
            SimView.targetTrajectory.clear();
    }

    public boolean isPreparing() {
        return isPreparing;
    }

    public boolean isReadyToShoot() {
        return isPreparing && flywheel.isReadyToShoot();
    }

    public void setShouldRunFlywheel(boolean shouldRunFlywheel) {
        this.shouldRunFlywheel = shouldRunFlywheel;
    }

    public double getFlywheelPercentOfTarget() {
        return flywheel.getPercentOfTarget();
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
