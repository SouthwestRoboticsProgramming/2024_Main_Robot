package com.swrobotics.robot.subsystems.speaker;

import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.subsystems.speaker.aim.AimCalculator;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Stow
 * Calibrate
 * Prep
 * Shoot
 * High Lob
 * Low Lob
 * Amp
 */

public class ShooterSubsystem2 extends SubsystemBase {
    public static final record ShootingState(Translation2d targetRelativeVelocity, double distanceToTarget, double currentFlywheelVelocityMPS) {}

    private static final Pose2d blueSpeakerPose = new Pose2d(Units.inchesToMeters(6), 5.5475, new Rotation2d(0)); // Opening extends 18" out

    private final PivotSubsystem pivot;
    private final FlywheelSubsystem flywheel;

    private final SwerveDrive drive;
    private final IndexerSubsystem indexer;

    public ShooterSubsystem2(SwerveDrive drive, IndexerSubsystem indexer) {
        this.pivot = new PivotSubsystem();
        this.flywheel = new FlywheelSubsystem();

        this.drive = drive;
        this.indexer = indexer;

        setDefaultCommand(getCalibratedIdleCommand());
    }

    private Pose2d getSpeakerPose() {
        return drive.getFieldInfo().flipPoseForAlliance(blueSpeakerPose);
    }

    private Command getCalibratedIdleCommand() {
        Command cmd = Commands.run(() -> pivot.setIdle())
            .alongWith(Commands.run(() -> flywheel.setNeutral()));
        cmd.addRequirements(this);
        return cmd;
    }

    public Command getSpeakerCommand() {
        Command cmd = getStaticAimCommand(getSpeakerPose());
        cmd.addRequirements(this);
        return cmd;
    }

    public Command getAmpCommand() {
        Command cmd = Commands.run(() -> pivot.setTargetAngle(Units.degreesToRotations(NTData.SHOOTER_AMP_ANGLE.get())))
        .alongWith(Commands.run(() -> flywheel.setTargetVelocity(NTData.SHOOTER_AMP_VELOCITY.get())));
        cmd.addRequirements(this);
        return cmd;
    }

    private Command getStaticAimCommand(Pose2d target) {
        return drive.getAimCommand(() -> getAngleToPose(target));
    }

    private Rotation2d getAngleToPose(Pose2d pose) {
        return pose.getTranslation().minus(drive.getEstimatedPose().getTranslation()).getAngle();
    }

    private Translation2d getRelativeVelocityToPose(Pose2d pose) {
        Rotation2d angleToPose = getAngleToPose(pose);
        ChassisSpeeds robotSpeeds = drive.getFieldRelativeSpeeds();
        Translation2d robotVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        return robotVelocity.rotateBy(angleToPose);
    }

    @Override
    public void periodic() {
    
    }
}
