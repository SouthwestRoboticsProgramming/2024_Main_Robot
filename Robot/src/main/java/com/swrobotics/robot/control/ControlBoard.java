package com.swrobotics.robot.control;

import java.security.InvalidAlgorithmParameterException;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.swrobotics.lib.input.XboxController;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.commands.AimTowardsLobCommand;
import com.swrobotics.robot.commands.AimTowardsSpeakerCommand;
import com.swrobotics.robot.commands.SnapDistanceCommand;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.subsystems.climber.ClimberArm;
import com.swrobotics.robot.subsystems.speaker.IntakeSubsystem;
import com.swrobotics.robot.subsystems.speaker.PivotSubsystem;
import com.swrobotics.robot.subsystems.speaker.ShooterSubsystem;
import com.swrobotics.robot.subsystems.speaker.aim.AmpAimCalculator;
import com.swrobotics.robot.subsystems.speaker.aim.LobCalculator;
import com.swrobotics.robot.subsystems.speaker.aim.TableAimCalculator;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlBoard extends SubsystemBase {
    /**
     * Driver:
     * Left stick: translation
     * Right stick X: rotation
     * Left bumper: robot relative
     * Right trigger: aim at speaker
     * Right bumper: spin up flywheel
     * Dpad Left: Manual subwoofer shot
     * Dpad Right: Manual podium shot
     * Dpad Up: Snap to distance closer to speaker
     * Dpad Down: Snap to distance farther to speaker
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

    private final Debouncer driverSlowDebounce = new Debouncer(0.075);
    private final Debouncer driverRobotRelDebounce = new Debouncer(0.075);
    private final Debouncer driverFlywheelDebounce = new Debouncer(0.075);
    private final Debouncer driverSnapCloserDebounce = new Debouncer(0.1);
    private final Debouncer driverSnapFartherDebounce = new Debouncer(0.1);
    private final Debouncer forceSubwooferDebounce = new Debouncer(0.1);
    private final Debouncer forceStageCornerDebounce = new Debouncer(0.1);
    private final Debouncer intakeDebounce = new Debouncer(0.075);
    private final Debouncer shootDebounce = new Debouncer(0.075);

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

        new Trigger(driver.leftBumper::isPressed)
            .whileTrue(new AimTowardsLobCommand(robot.drive, robot.shooter))
            .whileTrue(Commands.run(() -> robot.shooter.setTempAimCalculator(LobCalculator.INSTANCE)))
            .debounce(0.2, DebounceType.kRising) // Only debounce the shooting
            .onFalse(
                Commands.run(() -> robot.indexer.setFeedToShooter(true)).withTimeout(0.5)
                .andThen(Commands.runOnce(() -> robot.indexer.setFeedToShooter(false))));

        new Trigger(this::driverWantsAim).whileTrue(new AimTowardsSpeakerCommand(
                robot.drive,
                robot.shooter
        ));

        // Up is closer, down is farther
        new Trigger(this::driverWantsSnapCloser).whileTrue(new SnapDistanceCommand(robot.drive, robot.shooter, true));
        new Trigger(this::driverWantsSnapFarther).whileTrue(new SnapDistanceCommand(robot.drive, robot.shooter, false));

//        Trigger ampTrigger = new Trigger(() -> operator.y.isPressed());
//        ampTrigger.whileTrue(Commands.run(() -> robot.shooter.setTempAimCalculator(new AmpAimCalculator())));
//        ampTrigger.onFalse(new ShootCommand(robot));

        climberState = ClimberArm.State.RETRACTED;

//        Trigger operatorA = new Trigger(operator.a::isPressed);
//        operatorA.onTrue(Commands.runOnce(() -> robot.intake.set(IntakeSubsystem.State.INTAKE)));
        operator.a.onRising(() -> robot.intake.set(IntakeSubsystem.State.INTAKE));
        operator.a.onFalling(() -> robot.intake.set(IntakeSubsystem.State.OFF));
//        operatorA.onFalse(Commands.runOnce(() -> robot.intake.set(IntakeSubsystem.State.OFF)));
        robot.intake.set(IntakeSubsystem.State.OFF);

//        operator.a.onFalling(() -> robot.intake.set(IntakeSubsystem.State.INTAKE));
//        operator.a.onRising(() -> robot.intake.set(IntakeSubsystem.State.OFF));
        new Trigger(() -> robot.indexer.hasPiece() || !operator.a.isPressed())
                .onTrue(Commands.runOnce(() -> robot.intake.set(IntakeSubsystem.State.OFF)));
    }

    private boolean driverWantsSnapCloser() {
        return driverSnapCloserDebounce.calculate(driver.dpad.up.isPressed());
    }

    private boolean driverWantsSnapFarther() {
        return driverSnapFartherDebounce.calculate(driver.dpad.down.isPressed());
    }

    private boolean driverWantsAim() {
        return driver.rightTrigger.isOutside(TRIGGER_BUTTON_THRESHOLD)
                || driverWantsSnapCloser()
                || driverWantsSnapFarther();
    }

    private boolean driverWantsFlywheels() {
        return driverFlywheelDebounce.calculate(driver.rightBumper.isPressed());
    }

    private double squareWithSign(double value) {
        double squared = value * value;
        return Math.copySign(squared, value);
    }

    private double powerWithSign(double value, double power) {
        double powered = Math.pow(Math.abs(value), power);
        return Math.copySign(powered, value);
    }

    private Translation2d getDriveTranslation() {
        // double speed = NTData.DRIVE_SPEED_NORMAL.get();
        double speed = SwerveDrive.MAX_LINEAR_SPEED;

        Translation2d leftStick = driver.getLeftStick();
        // double x = -squareWithSign(leftStick.getY()) * speed;
        // double y = -squareWithSign(leftStick.getX()) * speed;
        double power = 2.5;
        double x = -powerWithSign(leftStick.getY(), power) * speed;
        double y = -powerWithSign(leftStick.getX(), power) * speed;
        return new Translation2d(x, y);
    }

    private double getDriveRotation() {
        return -squareWithSign(driver.rightStickX.get()) * NTData.TURN_SPEED.get();
    }

    private boolean getRobotRelativeDrive() {
        // return driverRobotRelDebounce.calculate(driver.leftBumper.isPressed());
        return false;
    }

    @Override
    public void periodic() {
        if (!DriverStation.isTeleop()) {
            climberState = ClimberArm.State.RETRACTED;
            robot.indexer.setReverse(false);
            robot.intake.setReverse(false);
            robot.shooter.setFlywheelControl(ShooterSubsystem.FlywheelControl.SHOOT); // Always aim with note when not teleop
            return;
        }


        boolean forceToSubwoofer = forceSubwooferDebounce.calculate(driver.dpad.left.isPressed());
        boolean forceToStageCorner = forceStageCornerDebounce.calculate(driver.dpad.right.isPressed());
        robot.drive.setEstimatorIgnoreVision(forceToSubwoofer || forceToStageCorner);

        if (forceToSubwoofer)
            robot.drive.setPose(robot.drive.getFieldInfo().flipPoseForAlliance(new Pose2d(1.393, 5.512, new Rotation2d(Math.PI))));
        if (forceToStageCorner)
            robot.drive.setPose(robot.drive.getFieldInfo().flipPoseForAlliance(new Pose2d(3.354, 5.512, new Rotation2d(Math.PI))));


        NTEntry<Double> pivotAdjust = PivotSubsystem.getAdjustForAlliance();
        if (operator.dpad.up.isRising())
            pivotAdjust.set(pivotAdjust.get() + 1);
        if (operator.dpad.down.isRising())
            pivotAdjust.set(pivotAdjust.get() - 1);


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

        // Indexer uses the intake state also
        boolean operatorWantsShoot = shootDebounce.calculate(operator.b.isPressed());
        robot.indexer.setFeedToShooter(operatorWantsShoot);

//        AmpArmSubsystem.Position ampArmPosition = AmpArmSubsystem.Position.STOW;
//        AmpIntakeSubsystem.State ampIntakeState = AmpIntakeSubsystem.State.OFF;
//        if (operator.x.isPressed()) {
//            ampArmPosition = AmpArmSubsystem.Position.PICKUP;
//            ampIntakeState = AmpIntakeSubsystem.State.INTAKE;
//        } else if (operator.y.isPressed()) {
//            ampArmPosition = AmpArmSubsystem.Position.SCORE_AMP;
//        } else if (operator.leftBumper.isPressed()) {
//            ampArmPosition = AmpArmSubsystem.Position.SCORE_TRAP;
//        }
//        if (operator.leftTrigger.isOutside(TRIGGER_BUTTON_THRESHOLD)) {
//            ampIntakeState = AmpIntakeSubsystem.State.OUTTAKE;
//        }
//        robot.ampArm.setPosition(ampArmPosition);
//        robot.ampIntake.setState(ampIntakeState);

        if (operator.rightBumper.isRising()) {
            boolean extended = climberState == ClimberArm.State.EXTENDED;
            climberState = extended ? ClimberArm.State.RETRACTED : ClimberArm.State.EXTENDED;
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


        double distToSpeaker = robot.shooter.getSpeakerPosition().getDistance(robot.drive.getEstimatedPose().getTranslation());
        boolean tooFar = TableAimCalculator.INSTANCE.isTooFar(distToSpeaker);

        driver.setRumble(pieceRumble ? 0.6 : (tooFar && (driverWantsAim() || driverWantsFlywheels()) ? 0.5 : 0));
        boolean shooterReady = robot.shooter.isReadyToShoot();
        operator.setRumble(pieceRumble ? 0.6 : (shooterReady ? 0.5 : 0));
    }

    public void setPieceRumble(boolean pieceRumble) {
        this.pieceRumble = pieceRumble;
    }
}
