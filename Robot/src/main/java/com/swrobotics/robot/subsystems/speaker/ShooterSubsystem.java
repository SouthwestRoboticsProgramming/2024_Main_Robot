package com.swrobotics.robot.subsystems.speaker;

import com.swrobotics.lib.net.NTString;
import edu.wpi.first.wpilibj.RobotBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.subsystems.speaker.aim.AimCalculator;
import com.swrobotics.robot.subsystems.speaker.aim.LobCalculator;
import com.swrobotics.robot.subsystems.speaker.aim.TableAimCalculator;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ShooterSubsystem extends SubsystemBase {
    private static final Pose2d blueSpeakerPose = new Pose2d(Units.inchesToMeters(6), 5.5475, new Rotation2d(0)); // Opening
                                                                                                                  // extends
                                                                                                                  // 18"
                                                                                                                  // out

    private final PivotSubsystem pivot;
    private final FlywheelSubsystem flywheel;

    private final SwerveDrive drive;
    private final IndexerSubsystem indexer;

    private Debouncer afterShootDelay;
    private boolean isPreparing;

    private AimCalculator.Aim targetAim; // Target aim is null if not currently aiming

    public ShooterSubsystem(SwerveDrive drive, IndexerSubsystem indexer) {
        this.pivot = new PivotSubsystem();
        this.flywheel = new FlywheelSubsystem();

        this.drive = drive;
        this.indexer = indexer;

        NTData.SHOOTER_AFTER_DELAY
                .nowAndOnChange((delay) -> afterShootDelay = new Debouncer(delay, Debouncer.DebounceType.kFalling));

        setDefaultCommand(getIdleCommand().ignoringDisable(true));
        handleIdle();
    }

    public Translation2d getSpeakerPosition() {
        return drive.getFieldInfo().flipPoseForAlliance(blueSpeakerPose).getTranslation();
    }

    public Translation2d getLobZonePosition() {
        return drive.getFieldInfo().flipPoseForAlliance(FieldView.lobZone.getPose()).getTranslation();
    }

    public void forcePivotCalibration(double angleDeg) {
        pivot.overrideCalibration(angleDeg);
    }

    private final NTString activeMode = new NTString("Shooter/Mode", "none");

    @Override
    public void periodic() {
        // whichAimCalculator.set(aimCalculator.getClass().getSimpleName());

        // // Use the selected aim calculator
        // AimCalculator.Aim aim;
        // if (aimCalculator instanceof LobCalculator) {
        // double distToLob =
        // getLobZonePosition().getDistance(drive.getEstimatedPose().getTranslation());
        // Pose2d robotPose = drive.getEstimatedPose();
        // Translation2d robotPos = robotPose.getTranslation();
        // ChassisSpeeds robotSpeeds = drive.getFieldRelativeSpeeds();

        // Translation2d target = getLobZonePosition();
        // Rotation2d angleToTarget = target.minus(robotPos).getAngle();

        // // Relative to the target
        // Translation2d robotVelocity = new
        // Translation2d(robotSpeeds.vxMetersPerSecond,
        // robotSpeeds.vyMetersPerSecond).rotateBy(angleToTarget);
        // aim = LobCalculator.INSTANCE.calculateAim(distToLob, robotVelocity.getX());

        // } else {
        // double distToSpeaker =
        // getSpeakerPosition().getDistance(drive.getEstimatedPose().getTranslation());
        // Pose2d robotPose = drive.getEstimatedPose();
        // Translation2d robotPos = robotPose.getTranslation();
        // ChassisSpeeds robotSpeeds = drive.getFieldRelativeSpeeds();

        // Translation2d target = getSpeakerPosition();
        // Rotation2d angleToTarget = target.minus(robotPos).getAngle();

        // // Relative to the target
        // Translation2d robotVelocity = new
        // Translation2d(robotSpeeds.vxMetersPerSecond,
        // robotSpeeds.vyMetersPerSecond).rotateBy(angleToTarget);
        // if (aimCalculator instanceof TableAimCalculator)
        // aim = ((TableAimCalculator) aimCalculator).calculateAim(distToSpeaker,
        // robotVelocity.getX());
        // else
        // aim = aimCalculator.calculateAim(distToSpeaker);

        // if (RobotBase.isSimulation())
        // SimView.lobTrajectory.update(
        // aim.flywheelVelocity() / NTData.SHOOTER_LOB_POWER_COEFFICIENT.get(),
        // aim.pivotAngle());
        // SimView.lobTrajectory.clear();
        // }
        // targetAim = aim;
        // aimCalculator = tableAimCalculator;

        // aimAngle.set(targetAim.pivotAngle());
        // aimVelocity.set(targetAim.flywheelVelocity());

        // isPreparing = false;
        // if (DriverStation.isDisabled() || !pivot.hasCalibrated())
        // return;

        // if (DriverStation.isAutonomous() && (aim != null)) {
        //     // Have the shooter be constantly active during auto
        //     isPreparing = true;
        //     flywheel.setTargetVelocity(aim.flywheelVelocity());
        //     pivot.setTargetAngle(aim.pivotAngle() / MathUtil.TAU);
        // } else if (flywheelControl == FlywheelControl.REVERSE) {
        //     pivot.setTargetAngle(NTData.SHOOTER_PIVOT_REVERSE_ANGLE.get() / 360.0);
        //     flywheel.setDutyCycle(-NTData.SHOOTER_FLYWHEEL_REVERSE_SPEED.get());
        // } else if (flywheelControl == FlywheelControl.SHOOT ||
        //         afterShootDelay.calculate(indexer.hasPiece())) {
        //     if (aim != null) {
        //         isPreparing = true;
        //         if (flywheelControl == FlywheelControl.SHOOT)
        //             flywheel.setTargetVelocity(aim.flywheelVelocity());
        //         else if (flywheelControl == FlywheelControl.POOP)
        //             flywheel.setDutyCycle(NTData.SHOOTER_FLYWHEEL_POOP_SPEED.get());
        //         else
        //             flywheel.setDutyCycle(NTData.SHOOTER_FLYWHEEL_IDLE_SPEED.get());
        //         pivot.setTargetAngle(aim.pivotAngle() / MathUtil.TAU);
        //     } else {
        //         if (flywheelControl == FlywheelControl.POOP)
        //             flywheel.setDutyCycle(NTData.SHOOTER_FLYWHEEL_POOP_SPEED.get());
        //         else
        //             flywheel.setDutyCycle(NTData.SHOOTER_FLYWHEEL_IDLE_SPEED.get());
        //         pivot.setIdle();
        //     }
        // } else if (flywheelControl == FlywheelControl.POOP) {
        //     flywheel.setDutyCycle(NTData.SHOOTER_FLYWHEEL_POOP_SPEED.get());
        //     pivot.setNeutral();
        // } else {
        //     flywheel.setNeutral();
        //     pivot.setNeutral();
        // }

        NTData.SHOOTER_READY.set(isReadyToShoot());
        pctErr.set(flywheel.getPercentErr());

        if (getCurrentCommand() != null) {
            activeMode.set(getCurrentCommand().getName());
        } else {
            activeMode.set("none");
        }
    }

    private void handleSpeaker() {
        double distToSpeaker = getSpeakerPosition().getDistance(drive.getEstimatedPose().getTranslation());
        Pose2d robotPose = drive.getEstimatedPose();
        Translation2d robotPos = robotPose.getTranslation();
        ChassisSpeeds robotSpeeds = drive.getFieldRelativeSpeeds();

        Translation2d target = getSpeakerPosition();
        Rotation2d angleToTarget = target.minus(robotPos).getAngle();

        // Relative to the target
        Translation2d robotVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond,
                robotSpeeds.vyMetersPerSecond).rotateBy(angleToTarget);
        AimCalculator.Aim aim = TableAimCalculator.INSTANCE.calculateAim(distToSpeaker, robotVelocity.getX());
        if (/* afterShootDelay.calculate(indexer.hasPiece()) && */(aim != null)) {
            pivot.setTargetAngle(aim.pivotAngle() / MathUtil.TAU);
            flywheel.setTargetVelocity(aim.flywheelVelocity());
            targetAim = aim;
        } else {
            handleIdle();
        }
    }

    public Command getSpeakerCommand() {
        return Commands.run(() -> handleSpeaker(), this).withName("Shooter Speaker");
    }

    private void handleIdle() {
        pivot.setNeutral();
        flywheel.setNeutral();
        targetAim = new AimCalculator.Aim(10.0, 22 / 360.0, 0);
    }

    public Command getIdleCommand() {
        return Commands.run(() -> handleIdle(), this).withName("Shooter Idle");
    }

    private void handleAmp() {
        pivot.setTargetAngle(Math.toRadians(NTData.SHOOTER_AMP_ANGLE.get()) / MathUtil.TAU);
        flywheel.setTargetVelocity(NTData.SHOOTER_AMP_VELOCITY.get());
    }

    public Command getAmpCommand() {
        return Commands.run(() -> handleAmp(), this).withName("Shooter Amp");
    }

    private void handleLob() {
        double distanceToTarget = distanceTo(getLobZonePosition());
        boolean chooseLowLob = distanceToTarget < 7.0 && drive.getEstimatedPose().getTranslation().getY() > 5.0; // Meters
        if (chooseLowLob) {
            pivot.setTargetAngle(25.0 / 360.0);
            flywheel.setDutyCycle(30);
            return;
        }

        AimCalculator.Aim aim = LobCalculator.INSTANCE.calculateAim(distanceToTarget);
        pivot.setTargetAngle(aim.pivotAngle() / MathUtil.TAU);
        flywheel.setTargetVelocity(aim.flywheelVelocity());
    }

    public Command getLobCommand() {
        return Commands.run(() -> handleLob(), this).withName("Shooter Lob");
    }

    private void handleFeed() {
        pivot.setTargetAngle(NTData.SHOOTER_PIVOT_REVERSE_ANGLE.get() / 360.0);
        flywheel.setDutyCycle(-NTData.SHOOTER_FLYWHEEL_REVERSE_SPEED.get());
    }

    public Command getFeedCommand() {
        return Commands.run(() -> handleFeed(), this).withName("Shooter Feed");
    }

    public Command getPoopCommand() {
        Command poop = Commands.run(() -> flywheel.setDutyCycle(NTData.SHOOTER_FLYWHEEL_POOP_SPEED.get()), this)
                .withName("Shooter Poop");

        // Every other command takes priority other than idle
        return Commands.either(poop.asProxy(), Commands.none(), () -> getCurrentCommand() == getDefaultCommand());
    }

    NTDouble pctErr = new NTDouble("Shooter/Debug/Percent Error", 0);

    public boolean isPreparing() {
        return isPreparing;
    }

    public boolean isReadyToShoot() {
        if (RobotBase.isSimulation()) {
            return true;
        }
        return isPreparing && flywheel.isReadyToShoot() && pivot.isAtSetpoint();
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

    public boolean isCalibrated() {
        return pivot.hasCalibrated();
    }

    private double distanceTo(Translation2d target) {
        return target.getDistance(drive.getEstimatedPose().getTranslation());
    }

    private Rotation2d angleTo(Translation2d target) {
        return target.minus(drive.getEstimatedPose().getTranslation()).getAngle();
    }

    private Translation2d velocityTo(Translation2d target) {
        ChassisSpeeds robotSpeeds = drive.getFieldRelativeSpeeds();
        return new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond).rotateBy(angleTo(target));
    }
}
