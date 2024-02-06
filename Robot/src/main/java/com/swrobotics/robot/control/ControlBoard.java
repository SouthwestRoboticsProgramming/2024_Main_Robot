package com.swrobotics.robot.control;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.swrobotics.lib.input.XboxController;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.RobotContainer;

import com.swrobotics.robot.commands.AimAtPointCommand;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.subsystems.amp.AmpArmSubsystem;
import com.swrobotics.robot.subsystems.amp.AmpIntakeSubsystem;
import com.swrobotics.robot.subsystems.speaker.IntakeSubsystem;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlBoard extends SubsystemBase {
    /**
     * Driver:
     * Left stick: translation
     * Right stick X: rotation
     * Left trigger: fast mode
     * Right trigger: aim at speaker
     *
     * Operator:
     * A: intake
     * B: shoot
     * X: amp intake
     * Y: amp score
     * Left trigger: eject
     * Left bumper: intake through shooter
     * Right trigger: amp eject
     *
     * Dpad up: climber up
     * Dpad down: climber down
     */

    private static final double DEADBAND = 0.15;
    private static final double TRIGGER_BUTTON_THRESHOLD = 0.3;

    private final RobotContainer robot;
    public final XboxController driver;
    public final XboxController operator;

    public ControlBoard(RobotContainer robot) {
        this.robot = robot;

        // Passing DEADBAND here means we don't have to deadband anywhere else
        driver = new XboxController(0, DEADBAND);
        operator = new XboxController(1, DEADBAND);

        // Pathing test
        Pose2d[] target = new Pose2d[1];
        target[0] = new Pose2d(10, 4, new Rotation2d(0));
        driver.a.onRising(() -> CommandScheduler.getInstance().schedule(AutoBuilder.pathfindToPose(target[0], new PathConstraints(0.5, 8, 10, 40))));
        driver.b.onRising(() -> target[0] = robot.drive.getEstimatedPose());

        // Configure triggers
        driver.start.onFalling(() -> robot.drive.setRotation(new Rotation2d()));
        driver.back.onFalling(() -> robot.drive.setRotation(new Rotation2d())); // Two buttons to reset gyro so the driver can't get confused

        new Trigger(() -> driver.rightTrigger.isOutside(TRIGGER_BUTTON_THRESHOLD)).whileTrue(new AimAtPointCommand(
                robot.drive,
                robot.shooter::getSpeakerPosition
        ));
    }

    private double squareWithSign(double value) {
        double squared = value * value;
        return Math.copySign(squared, value);
    }

    private Translation2d getDriveTranslation() {
        double speed = NTData.DRIVE_SPEED_NORMAL.get();
        if (driver.leftBumper.isPressed())
            speed = NTData.DRIVE_SPEED_SLOW.get();
        if (driver.leftTrigger.isOutside(TRIGGER_BUTTON_THRESHOLD))
            speed = SwerveDrive.MAX_LINEAR_SPEED;
//            speed = NTData.DRIVE_SPEED_FAST.get();

        Translation2d leftStick = driver.getLeftStick();
        double x = -squareWithSign(leftStick.getY()) * speed;
        double y = -squareWithSign(leftStick.getX()) * speed;
        return new Translation2d(x, y);
    }

    private double getDriveRotation() {
        return -squareWithSign(driver.rightStickX.get()) * NTData.TURN_SPEED.get();
    }

    @Override
    public void periodic() {
        if (!DriverStation.isTeleop())
            return;

        Translation2d translation = getDriveTranslation();
        double rawRotation = getDriveRotation();
        Rotation2d rotation = new Rotation2d(MathUtil.TAU * rawRotation);

        robot.drive.driveAndTurn(
                SwerveDrive.DRIVER_PRIORITY,
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation.getRadians(),
                        robot.drive.getEstimatedPose().getRotation()),
                DriveRequestType.Velocity);

        IntakeSubsystem.State intakeState = IntakeSubsystem.State.OFF;
        if (operator.a.isPressed())
            intakeState = IntakeSubsystem.State.INTAKE;
        else if (operator.leftTrigger.isOutside(TRIGGER_BUTTON_THRESHOLD))
            intakeState = IntakeSubsystem.State.EJECT;

        // Indexer uses the intake state also
        robot.intake.set(intakeState);
        robot.indexer.setFeedToShooter(operator.b.isPressed());

        AmpArmSubsystem.Position ampArmPosition = AmpArmSubsystem.Position.STOW;
        AmpIntakeSubsystem.State ampIntakeState = AmpIntakeSubsystem.State.OFF;
        if (operator.x.isPressed()) {
            ampArmPosition = AmpArmSubsystem.Position.PICKUP;
            ampIntakeState = AmpIntakeSubsystem.State.INTAKE;
        } else if (operator.y.isPressed()) {
            ampArmPosition = AmpArmSubsystem.Position.SCORE;
            if (operator.rightTrigger.isOutside(TRIGGER_BUTTON_THRESHOLD)) {
                ampIntakeState = AmpIntakeSubsystem.State.OUTTAKE;
            }
        }
        robot.ampArm.setPosition(ampArmPosition);
        robot.ampIntake.setState(ampIntakeState);
    }
}
