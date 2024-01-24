package com.swrobotics.robot.control;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.swrobotics.lib.input.XboxController;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.RobotContainer;

import com.swrobotics.robot.commands.AimAtPointCommand;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.subsystems.amp.AmpArmSubsystem;
import com.swrobotics.robot.subsystems.amp.AmpIntakeSubsystem;
import com.swrobotics.robot.subsystems.speaker.IndexerSubsystem;
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
     * Right trigger: amp eject
     *
     * Up: recalibrate shot higher
     * Down: recalibrate shot lower
     * Left trigger: recalibrate small step
     */

    private static final double DEADBAND = 0.15;

    /**
     * Manages demands to go to an exact angle demanded by the driver
     */
    public enum SwerveCardinal {
        NONE(0),
        FRONT(0),
        LEFT(90),
        RIGHT(-90),
        BACk(180);

        public final double degrees;

        SwerveCardinal(double degrees) {
            this.degrees = degrees;
        }
    }

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
        driver.a.onRising(() -> CommandScheduler.getInstance().schedule(AutoBuilder.pathfindToPose(target[0], new PathConstraints(4, 8, 10, 40))));
        driver.b.onRising(() -> target[0] = robot.drive.getEstimatedPose());

        // Congigure triggers
        driver.start.onFalling(() -> robot.drive.setRotation(new Rotation2d()));
        driver.back.onFalling(() -> robot.drive.setRotation(new Rotation2d())); // Two buttons to reset gyro so the driver can't get confused

        new Trigger(() -> driver.rightTrigger.get() != 0).whileTrue(new AimAtPointCommand(
                robot.drive,
                robot.shooter::getSpeakerPosition
        ));
    }

    public Translation2d getDriveTranslation() {
        double speed = NTData.DRIVE_SPEED_NORMAL.get();
        if (driver.leftBumper.isPressed())
            speed = NTData.DRIVE_SPEED_SLOW.get();
        if (driver.leftTrigger.get() != 0)
            speed = NTData.DRIVE_SPEED_FAST.get();

        Translation2d leftStick = driver.getLeftStick();
        double x = -leftStick.getY() * speed;
        double y = -leftStick.getX() * speed;
        return new Translation2d(x, y);
    }

    public double getDriveRotation() {
        return -driver.rightStickX.get() * NTData.TURN_SPEED.get();
    }

    @Override
    public void periodic() {
        if (!DriverStation.isTeleop())
            return;

        Translation2d translation = getDriveTranslation();
        double rawRotation = getDriveRotation();
        Rotation2d rotation = new Rotation2d(MathUtil.TAU * rawRotation);

        robot.drive.driveAndTurn(
                SwerveDrive.DEFAULT_PRIORITY,
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation.getRadians(),
                        robot.drive.getEstimatedPose().getRotation()),
                SwerveModule.DriveRequestType.OpenLoopVoltage);

        robot.intake.set(operator.a.isPressed());
        robot.indexer.setState(operator.b.isPressed() ? IndexerSubsystem.State.FEED_TO_SHOOTER : IndexerSubsystem.State.TAKE_FROM_INTAKE);

        AmpArmSubsystem.Position armPosition = AmpArmSubsystem.Position.STOW;
        AmpIntakeSubsystem.State intakeState = AmpIntakeSubsystem.State.OFF;
        if (operator.x.isPressed()) {
            armPosition = AmpArmSubsystem.Position.PICKUP;
            intakeState = AmpIntakeSubsystem.State.INTAKE;
        } else if (operator.y.isPressed()) {
            armPosition = AmpArmSubsystem.Position.SCORE;
            if (operator.rightTrigger.get() != 0) {
                intakeState = AmpIntakeSubsystem.State.OUTTAKE;
            }
        }
        robot.ampArm.setPosition(armPosition);
        robot.ampIntake.setState(intakeState);
    }
}
