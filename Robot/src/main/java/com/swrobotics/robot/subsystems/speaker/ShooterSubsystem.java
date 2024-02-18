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

// TODO: Split flywheel and pivot
public final class ShooterSubsystem extends SubsystemBase {
    private static final Pose2d blueSpeakerPose = new Pose2d(0, 5.5475, new Rotation2d(0));

    private final PivotSubsystem pivot;
    private final FlywheelSubsystem flywheel;

    private final SwerveDrive drive;
    private final IndexerSubsystem indexer;

    private Debouncer afterShootDelay;
    private boolean isPreparing;

    private AimCalculator.Aim targetAim;
    private final AimCalculator aimCalculator;

    public ShooterSubsystem(SwerveDrive drive, IndexerSubsystem indexer) {
        this.pivot = new PivotSubsystem();
        this.flywheel = new FlywheelSubsystem();

        this.drive = drive;
        this.indexer = indexer;

//        aimCalculator = new ManualAimCalculator();
        aimCalculator = new TableAimCalculator();

        NTData.SHOOTER_AFTER_DELAY.nowAndOnChange((delay) -> afterShootDelay = new Debouncer(delay, Debouncer.DebounceType.kFalling));
    }

    public Translation2d getSpeakerPosition() {
        return drive.getFieldInfo().flipPoseForAlliance(blueSpeakerPose).getTranslation();
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled() || !pivot.hasCalibrated())
            return;

        isPreparing = false;
        if (afterShootDelay.calculate(indexer.hasPiece())) {
            double distToSpeaker = getSpeakerPosition().getDistance(drive.getEstimatedPose().getTranslation());

            AimCalculator.Aim aim = aimCalculator.calculateAim(distToSpeaker);
            if (aim != null) {
                targetAim = aim;
                isPreparing = true;
                flywheel.setTargetVelocity(aim.flywheelVelocity());
                pivot.setTargetAngle(aim.pivotAngle() / MathUtil.TAU);
            } else {
                targetAim = null;
                flywheel.setIdle();
                pivot.setIdle();
            }
        } else {
            targetAim = null;
            flywheel.setNeutral();
            pivot.setNeutral();
        }

        ready.set(isReadyToShoot());
        pctErr.set(flywheel.getPercentErr());
    }

    NTBoolean ready = new NTBoolean("Shooter/Debug/Flywheel Ready", false);
    NTDouble pctErr = new NTDouble("Shooter/Debug/Percent Error", 0);

    @Override
    public void simulationPeriodic() {
        if (targetAim == null)
            SimView.targetTrajectory.clear();
        else
            SimView.targetTrajectory.update(targetAim);
    }

    public boolean isPreparing() {
        return isPreparing;
    }

    public boolean isReadyToShoot() {
        return isPreparing && flywheel.isReadyToShoot();
    }

    public double getFlywheelPercentOfTarget() {
        return flywheel.getPercentOfTarget();
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
