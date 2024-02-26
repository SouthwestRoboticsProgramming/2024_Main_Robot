package com.swrobotics.robot.control;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.swrobotics.lib.input.XboxController;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.RobotContainer;

import com.swrobotics.robot.commands.AimTowardsSpeakerCommand;
import com.swrobotics.robot.commands.ShootCommand;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.subsystems.amp.AmpArmSubsystem;
import com.swrobotics.robot.subsystems.amp.AmpIntakeSubsystem;
import com.swrobotics.robot.subsystems.climber.ClimberArm;
import com.swrobotics.robot.subsystems.speaker.IntakeSubsystem;
import com.swrobotics.robot.subsystems.speaker.aim.AmpAimCalculator;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlBoard extends SubsystemBase {
    /**
     * Driver:
     * Left stick: translation
     * Right stick X: rotation
     * Left trigger: fast mode
     * Left bumper: robot relative
     * Right trigger: aim at speaker
     * Right bumper: spin up flywheel
     * Dpad Left: Manual subwoofer shot
     * Dpad Right: Manual podium shot
     *
     * Operator:
     * A: intake
     * B: shoot
     * X: amp intake
     * Y: amp score [Duluth-Hold to prep amp, release to launch]
     * Left trigger: amp eject
     * Right bumper: toggle climber extend/retract
     * Right trigger: retract with feedforward
     * Back: intake eject
     * Start: indexer eject
     */

    private static final double DEADBAND = 0.15;
    private static final double TRIGGER_BUTTON_THRESHOLD = 0.3;

    private final RobotContainer robot;
    public final XboxController driver;
    public final XboxController operator;

    private ClimberArm.State climberState;

    public ControlBoard(RobotContainer robot) {
        this.robot = robot;

        // Passing DEADBAND here means we don't have to deadband anywhere else
        driver = new XboxController(0, DEADBAND);
        operator = new XboxController(1, DEADBAND);

        // Pathing test
        // Pose2d[] target = new Pose2d[1];
        // target[0] = new Pose2d(10, 4, new Rotation2d(0));
        // driver.a.onRising(() -> CommandScheduler.getInstance().schedule(AutoBuilder.pathfindToPose(target[0], new PathConstraints(0.5, 8, 10, 40))));
        // driver.b.onRising(() -> target[0] = robot.drive.getEstimatedPose());

        // Configure triggers
        driver.start.onFalling(() -> robot.drive.setRotation(new Rotation2d()));
        driver.back.onFalling(() -> robot.drive.setRotation(new Rotation2d())); // Two buttons to reset gyro so the driver can't get confused

        new Trigger(this::driverWantsAim).whileTrue(new AimTowardsSpeakerCommand(
                robot.drive,
                robot.shooter
        ));

        Trigger ampTrigger = new Trigger(() -> operator.y.isPressed());
        ampTrigger.whileTrue(Commands.run(() -> robot.shooter.setTempAimCalculator(new AmpAimCalculator())));
        ampTrigger.onFalse(new ShootCommand(robot));

        climberState = ClimberArm.State.RETRACTED_IDLE;
    }

    private boolean driverWantsAim() {
        return driver.rightTrigger.isOutside(TRIGGER_BUTTON_THRESHOLD);
    }

    private boolean driverWantsFlywheels() {
        return driver.rightBumper.isPressed();
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

    private boolean getRobotRelativeDrive() {
        return driver.leftBumper.isPressed();
    }

    @Override
    public void periodic() {
        if (!DriverStation.isTeleop()) {
            climberState = ClimberArm.State.RETRACTED_IDLE;
            robot.indexer.setReverse(false);
            robot.intake.setReverse(false);
            robot.shooter.setShouldRunFlywheel(true); // Always aim with note when not teleop
            return;
        }

        Translation2d translation = getDriveTranslation();
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            translation = translation.rotateBy(Rotation2d.fromDegrees(180));
        }

        double rawRotation = getDriveRotation();
        Rotation2d rotation = new Rotation2d(MathUtil.TAU * rawRotation);

        ChassisSpeeds chassisRequest = ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation.getRadians(),
                        robot.drive.getEstimatedPose().getRotation());

        if (getRobotRelativeDrive()) {
            chassisRequest = new ChassisSpeeds(-translation.getX(), -translation.getY(), rotation.getRadians());
        }

        robot.drive.driveAndTurn(
                SwerveDrive.DRIVER_PRIORITY,
                chassisRequest,
                DriveRequestType.Velocity);

        IntakeSubsystem.State intakeState = IntakeSubsystem.State.OFF;
        if (operator.a.isPressed())
            intakeState = IntakeSubsystem.State.INTAKE;

        // Indexer uses the intake state also
        robot.intake.set(intakeState);
        robot.indexer.setFeedToShooter(operator.b.isPressed());

        // Run the shooter a little when the operator wants to shooter but the driver doesn't (lets us poop a note out)
        new Trigger(() -> operator.b.isFalling() && !(driverWantsAim() || driverWantsFlywheels())).onTrue(
            Commands.run(() -> NTData.SHOOTER_FLYWHEEL_IDLE_SPEED.set(NTData.SHOOTER_FLYWHEEL_IDLE_SPEED.get() + 0.2)).withTimeout(0.5)
            .andThen(() -> NTData.SHOOTER_FLYWHEEL_IDLE_SPEED.set(NTData.SHOOTER_FLYWHEEL_IDLE_SPEED.get() - 0.2))
        );

        AmpArmSubsystem.Position ampArmPosition = AmpArmSubsystem.Position.STOW;
        AmpIntakeSubsystem.State ampIntakeState = AmpIntakeSubsystem.State.OFF;
        if (operator.x.isPressed()) {
            ampArmPosition = AmpArmSubsystem.Position.PICKUP;
            ampIntakeState = AmpIntakeSubsystem.State.INTAKE;
        } else if (operator.y.isPressed()) {
            ampArmPosition = AmpArmSubsystem.Position.SCORE_AMP;
            if (operator.leftTrigger.isOutside(TRIGGER_BUTTON_THRESHOLD)) {
                ampIntakeState = AmpIntakeSubsystem.State.OUTTAKE;
            }
        }
        robot.ampArm.setPosition(ampArmPosition);
        robot.ampIntake.setState(ampIntakeState);

        if (operator.rightBumper.isRising()) {
            boolean extended = climberState == ClimberArm.State.EXTENDED;
            climberState = extended ? ClimberArm.State.RETRACTED_IDLE : ClimberArm.State.EXTENDED;
        }
        if (operator.rightTrigger.isOutside(TRIGGER_BUTTON_THRESHOLD)) {
            climberState = ClimberArm.State.RETRACTED_HOLD;
        }
        robot.climber.setState(climberState);

        robot.intake.setReverse(operator.back.isPressed());
        robot.indexer.setReverse(operator.start.isPressed());

        robot.shooter.setShouldRunFlywheel(driverWantsAim() || driverWantsFlywheels());
    }
}
