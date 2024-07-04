package com.swrobotics.robot.control;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.swrobotics.lib.input.XboxController;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.commands.AimTowardsSpeakerCommand;
import com.swrobotics.robot.commands.AmpAlignCommand;
import com.swrobotics.robot.commands.CharactarizeWheelCommand;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.subsystems.amp.AmpArm2Subsystem;
import com.swrobotics.robot.subsystems.climber.ClimberArm;
import com.swrobotics.robot.subsystems.speaker.IntakeSubsystem;
import com.swrobotics.robot.subsystems.speaker.PivotSubsystem;
import com.swrobotics.robot.subsystems.speaker.ShooterSubsystem;
import com.swrobotics.robot.subsystems.speaker.aim.ManualAimCalculator;
import com.swrobotics.robot.subsystems.speaker.aim.TableAimCalculator;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive.TurnRequest;

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
     * Left bumper: high lob
     * Left Trigger: low lob
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
     * Y: aim at amp and amp bar
     * Left trigger: amp eject
     * Right bumper: press to extend, press to climb
     * Left bumper: press to cancel climb
     * Back: intake eject
     * Start: indexer eject
     */

    private static final double DEADBAND = 0.15;
    private static final double TRIGGER_BUTTON_THRESHOLD = 0.3;

    private static final NTEntry<Boolean> CHARACTERISE_WHEEL_RADIUS = new NTBoolean("Drive/Charactarize Wheel Radius", false);

    private final RobotContainer robot;
    public final XboxController driver;
    public final XboxController operator;

    private boolean pieceRumble;

    private static final double MAX_DRIVE_ACCEL = 5.5; // Meters / second^2
    private final DriveAccelFilter driveFilter = new DriveAccelFilter(MAX_DRIVE_ACCEL);

    public ControlBoard(RobotContainer robot) {
        this.robot = robot;

        // Passing DEADBAND here means we don't have to deadband anywhere else
        driver = new XboxController(0, DEADBAND);
        operator = new XboxController(1, DEADBAND);

        // Configure triggers

        // Two buttons to reset gyro so the driver can't get confused
        driver.start.onFalling(() -> robot.drive.setRotation(new Rotation2d()));
        driver.back.onFalling(() -> robot.drive.setRotation(new Rotation2d())); 

        // Aim button FIXME: Do better
        new Trigger(() -> driverWantsAim()).whileTrue(robot.shooter.getSpeakerCommand());

        // Spin like mad when the driver clicks in the right stick
        new Trigger(driver.rightStickButton::isPressed)
                .debounce(0.1)
                .whileTrue(Commands.run(() -> robot.drive.turn(new TurnRequest(SwerveDrive.DRIVER_PRIORITY + 1, new Rotation2d(11.0)))));


        // Angle towards the amp
        new Trigger(() -> driver.a.isPressed()).whileTrue(new AmpAlignCommand(robot.drive).alongWith());

        new Trigger(() -> operator.y.isPressed()).whileTrue(robot.shooter.getAmpCommand());

        // Intake (automatically rises when we obtain a game piece)
        operator.a.onRising(() -> robot.intake.set(IntakeSubsystem.State.INTAKE));
        operator.a.onFalling(() -> robot.intake.set(IntakeSubsystem.State.OFF));
        robot.intake.set(IntakeSubsystem.State.OFF);

        // Raise the intake when we have a piece
        new Trigger(() -> robot.indexer.hasPiece() || !operator.a.isPressed())
                .onTrue(Commands.runOnce(() -> robot.intake.set(IntakeSubsystem.State.OFF)));

        // Routine to fix odometry
        new Trigger(() -> CHARACTERISE_WHEEL_RADIUS.get()).whileTrue(new CharactarizeWheelCommand(robot.drive));

        // Fixme: More intuitive eject
        new Trigger(operator.start::isPressed)
                .onTrue(Commands.runOnce(robot.indexer::beginReverse))
                .onFalse(Commands.runOnce(robot.indexer::endReverse));

        new Trigger(() -> operator.b.isPressed()).debounce(0.04)
            .whileTrue(Commands.runOnce(() -> robot.indexer.setFeedToShooter(true)));

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
        double speed = SwerveDrive.MAX_LINEAR_SPEED;

        Translation2d leftStick = driver.getLeftStick();
        double power = 2.0;

        double rawMag = leftStick.getNorm();
        double powerMag = powerWithSign(rawMag, power);

        if (rawMag == 0 || powerMag == 0)
            return new Translation2d(0, 0); // No division by 0

        double targetSpeed = powerMag * speed;
        double filteredSpeed = driveFilter.calculate(targetSpeed);

        return new Translation2d(-leftStick.getY(), -leftStick.getX()).div(rawMag).times(filteredSpeed);
    }

    private double getDriveRotation() {
        return -squareWithSign(driver.rightStickX.get()) * NTData.TURN_SPEED.get();
    }

    private boolean driverWantsAim() {
        return driver.rightBumper.isPressed();
    }

    private enum ClimbState {
        IDLE(ClimberArm.State.RETRACTED, null),
        EXTENDED(ClimberArm.State.EXTENDED, AmpArm2Subsystem.Position.CLIMB_OUT_OF_THE_WAY),
        RETRACTED_ON_CHAIN(ClimberArm.State.RETRACTED, AmpArm2Subsystem.Position.CLIMB_OUT_OF_THE_WAY);

        final ClimberArm.State climberState;
        final AmpArm2Subsystem.Position ampArmPosOverride;

        ClimbState(ClimberArm.State climberState, AmpArm2Subsystem.Position ampArmPosOverride) {
            this.climberState = climberState;
            this.ampArmPosOverride = ampArmPosOverride;
        }
    }

    private ClimbState climbState = ClimbState.IDLE;
    private boolean waitingForArmRetract = false;

    @Override
    public void periodic() {
        if (!DriverStation.isTeleop()) {
            climbState = ClimbState.IDLE;
            robot.indexer.endReverse();
            robot.intake.setReverse(false);
            // robot.shooter.setFlywheelControl(ShooterSubsystem.FlywheelControl.SHOOT); // Always aim with note when not teleop

            // robot.drive.setEstimatorIgnoreVision(false && DriverStation.isAutonomous());

            return;
        }

        if (!operator.a.isPressed())
            robot.intake.set(IntakeSubsystem.State.OFF);


        // FIXME: Change
        // boolean forceToSubwoofer = forceSubwooferDebounce.calculate(driver.dpad.left.isPressed());
        // boolean forceToStageCorner = forceStageCornerDebounce.calculate(driver.dpad.right.isPressed());
        // robot.drive.setEstimatorIgnoreVision(forceToSubwoofer || forceToStageCorner);

        // if (forceToSubwoofer)
        //     robot.drive.setPose(robot.drive.getFieldInfo().flipPoseForAlliance(new Pose2d(1.393, 5.512, new Rotation2d(Math.PI))));
        // if (forceToStageCorner)
        //     robot.drive.setPose(robot.drive.getFieldInfo().flipPoseForAlliance(new Pose2d(3.354, 5.512, new Rotation2d(Math.PI))));


        NTEntry<Double> pivotAdjust = PivotSubsystem.getAdjustForAlliance();
        double adjustDeg = 0.75;
        if (operator.dpad.up.isRising())
            pivotAdjust.set(pivotAdjust.get() + adjustDeg);
        if (operator.dpad.down.isRising())
            pivotAdjust.set(pivotAdjust.get() - adjustDeg);


        double adjustPower = 0.2;
        if (driver.dpad.up.isRising())
            NTData.SHOOTER_LOB_POWER_COEFFICIENT.set(NTData.SHOOTER_LOB_POWER_COEFFICIENT.get() + adjustPower);
        if (driver.dpad.down.isRising())
            NTData.SHOOTER_LOB_POWER_COEFFICIENT.set(NTData.SHOOTER_LOB_POWER_COEFFICIENT.get() - adjustPower);


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

        robot.drive.driveAndTurn(
                SwerveDrive.DRIVER_PRIORITY,
                chassisRequest,
                DriveRequestType.Velocity);



        // robot.ampArm2.setOut(operator.y.isPressed());

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

        boolean climbToggle = operator.rightBumper.isRising();
        boolean climbCancel = operator.leftBumper.isRising();

        boolean wasExtended = climbState == ClimbState.EXTENDED;
        if (climbToggle && (climbState == ClimbState.IDLE || climbState == ClimbState.RETRACTED_ON_CHAIN)) {
            climbState = ClimbState.EXTENDED;
        } else if (climbToggle && climbState == ClimbState.EXTENDED) {
            climbState = ClimbState.RETRACTED_ON_CHAIN;
        }
        if (climbCancel)
            climbState = ClimbState.IDLE;

        if (climbState == ClimbState.IDLE && wasExtended)
          waitingForArmRetract = true;
        
        AmpArm2Subsystem.Position ampArmPos = operator.y.isPressed()
            ? AmpArm2Subsystem.Position.AMP : AmpArm2Subsystem.Position.RETRACT;
        if (climbState.ampArmPosOverride != null)
            ampArmPos = climbState.ampArmPosOverride;
        if (climbState == ClimbState.IDLE && waitingForArmRetract) {
            // DON'T COLLIDE WITH ARMS
            boolean armsDown = robot.climber.isAtPosition();
            if (armsDown)
               waitingForArmRetract = false;
            else
              ampArmPos = AmpArm2Subsystem.Position.CLIMB_OUT_OF_THE_WAY;
        } else {
            waitingForArmRetract = false;
        }

        if (ampArmPos == AmpArm2Subsystem.Position.AMP) {
            NTData.AMP_ARM_EXTEND_POS.set(NTData.AMP_ARM_EXTEND_POS.get() + -operator.leftStickY.get() * 10 / 20);
        }

        robot.ampArm2.setPosition(ampArmPos);
        robot.climber.setState(climbState.climberState);

        robot.climber.applyManualAdjust(-operator.leftStickY.get(), -operator.rightStickY.get());

        // if (operator.rightBumper.isRising()) {
        //     boolean extended = climbStateClimberArm.State.EXTENDED;
        //     climberState = extended ? ClimberArm.State.RETRACTED : ClimberArm.State.EXTENDED;
        // }
        // robot.climber.setState(climberState);

        robot.intake.setReverse(operator.back.isPressed());

        // boolean shootAmp = operator.y.isPressed();
        // if (shootAmp)
        //     robot.shooter.setTempAimCalculator(AmpAimCalculator.INSTANCE);
        // boolean shootManual = operator.x.isPressed();
        // if (shootManual)
        //     robot.shooter.setTempAimCalculator(ManualAimCalculator.INSTANCE);

//        robot.shooter.setFlywheelControl(driverWantsAim() || driverWantsFlywheels());
        ShooterSubsystem.FlywheelControl flywheelControl = ShooterSubsystem.FlywheelControl.IDLE;
        // if (operator.start.isPressed())
        //     flywheelControl = ShooterSubsystem.FlywheelControl.REVERSE;
        // else if (driverWantsAim() || shootAmp || shootManual)
        //     flywheelControl = ShooterSubsystem.FlywheelControl.SHOOT;
        // else if (operatorWantsShoot)
        //     flywheelControl = ShooterSubsystem.FlywheelControl.POOP;
        // robot.shooter.setFlywheelControl(flywheelControl);


        // double distToSpeaker = robot.shooter.getSpeakerPosition().getDistance(robot.drive.getEstimatedPose().getTranslation());
        // boolean tooFar = TableAimCalculator.INSTANCE.isTooFar(distToSpeaker);

        // pieceRumbleNt.set(pieceRumble);

        // driver.setRumble(pieceRumble ? 1.0 : (tooFar && (driverWantsAim()) ? 0.5 : 0));
        // boolean shooterReady = robot.shooter.isReadyToShoot();
        // operator.setRumble(pieceRumble ? 1.0 : (shooterReady ? 0.5 : 0));

        // this.shooterReady.set(shooterReady);
        // this.tooFar.set(tooFar && (driverWantsAim()));
    }

    NTBoolean pieceRumbleNt = new NTBoolean("Debug/Piece Rumble", false);
    NTBoolean shooterReady = new NTBoolean("Debug/Shooter Ready", false);
    NTBoolean tooFar = new NTBoolean("Debug/Too Far and Want Shoot", false);

    public boolean isClimbing() {
        return climbState != ClimbState.IDLE;
    }

    public void setPieceRumble(boolean pieceRumble) {
        this.pieceRumble = pieceRumble;
    }
}
