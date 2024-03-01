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
import com.swrobotics.robot.subsystems.speaker.ShooterSubsystem;
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
     * Dpad down: recalibrate pivot
     *
     * Operator:
     * A: intake
     * B: shoot
     * X: amp intake
     * Y: aim at amp
     * [disabled] Y: amp score
     * [disabled] Left bumper: amp score in trap
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
    private boolean pieceRumble;

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

//        Trigger ampTrigger = new Trigger(() -> operator.y.isPressed());
//        ampTrigger.whileTrue(Commands.run(() -> robot.shooter.setTempAimCalculator(new AmpAimCalculator())));
//        ampTrigger.onFalse(new ShootCommand(robot));

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
            robot.shooter.setFlywheelControl(ShooterSubsystem.FlywheelControl.SHOOT); // Always aim with note when not teleop
            return;
        }


        boolean forceToSubwoofer = driver.dpad.right.isPressed();
        boolean forceToStageCorner = driver.dpad.left.isPressed();
        robot.drive.setEstimatorIgnoreVision(forceToSubwoofer || forceToStageCorner);

        if (forceToSubwoofer)
            robot.drive.setPose(robot.drive.getFieldInfo().flipPoseForAlliance(new Pose2d(1.393, 5.512, new Rotation2d(Math.PI))));
        if (forceToStageCorner)
            robot.drive.setPose(robot.drive.getFieldInfo().flipPoseForAlliance(new Pose2d(3.354, 5.512, new Rotation2d(Math.PI))));


        driver.setRumble(pieceRumble ? 0.5 : 0);
        boolean shooterReady = robot.shooter.isReadyToShoot();
        operator.setRumble(pieceRumble ? 0.5 : (shooterReady ? 0.3 : 0));

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
        boolean operatorWantsShoot = operator.b.isPressed();
        robot.indexer.setFeedToShooter(operatorWantsShoot);

        AmpArmSubsystem.Position ampArmPosition = AmpArmSubsystem.Position.STOW;
        AmpIntakeSubsystem.State ampIntakeState = AmpIntakeSubsystem.State.OFF;
        if (operator.x.isPressed()) {
            ampArmPosition = AmpArmSubsystem.Position.PICKUP;
            ampIntakeState = AmpIntakeSubsystem.State.INTAKE;
        } else if (operator.y.isPressed()) {
            ampArmPosition = AmpArmSubsystem.Position.SCORE_AMP;
        } else if (operator.leftBumper.isPressed()) {
            ampArmPosition = AmpArmSubsystem.Position.SCORE_TRAP;
        }
        if (operator.leftTrigger.isOutside(TRIGGER_BUTTON_THRESHOLD)) {
            ampIntakeState = AmpIntakeSubsystem.State.OUTTAKE;
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

        boolean shootAmp = operator.y.isPressed();
        if (shootAmp)
            robot.shooter.setTempAimCalculator(AmpAimCalculator.INSTANCE);

//        robot.shooter.setFlywheelControl(driverWantsAim() || driverWantsFlywheels());
        ShooterSubsystem.FlywheelControl flywheelControl = ShooterSubsystem.FlywheelControl.IDLE;
        if (operator.start.isPressed())
            flywheelControl = ShooterSubsystem.FlywheelControl.REVERSE;
        else if (driverWantsAim() || driverWantsFlywheels() || shootAmp || forceToSubwoofer || forceToStageCorner)
            flywheelControl = ShooterSubsystem.FlywheelControl.SHOOT;
        else if (operatorWantsShoot)
            flywheelControl = ShooterSubsystem.FlywheelControl.POOP;
        robot.shooter.setFlywheelControl(flywheelControl);
    }

    public void setPieceRumble(boolean pieceRumble) {
        this.pieceRumble = pieceRumble;
    }
}
