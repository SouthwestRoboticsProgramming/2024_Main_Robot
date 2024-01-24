package com.swrobotics.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.messenger.client.MessengerClient;
import com.swrobotics.robot.control.ControlBoard;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.subsystems.amp.AmpArmSubsystem;
import com.swrobotics.robot.subsystems.amp.AmpIntakeSubsystem;
import com.swrobotics.robot.subsystems.climber.ClimberSubsystem;
import com.swrobotics.robot.subsystems.speaker.IndexerSubsystem;
import com.swrobotics.robot.subsystems.speaker.IntakeSubsystem;
import com.swrobotics.robot.subsystems.speaker.ShooterSubsystem;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive.DriveRequest;
import com.swrobotics.taskmanager.filesystem.FileSystemAPI;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Configuration for our Raspberry Pi communication service
    private static final String MESSENGER_HOST_ROBOT = "10.21.29.3";
    private static final String MESSENGER_HOST_SIM = "localhost";
    private static final int MESSENGER_PORT = 5805;
    private static final String MESSENGER_NAME = "Robot";

    // Create a way to choose between autonomous sequences
    private final LoggedDashboardChooser<Command> autoSelector;

    public final MessengerClient messenger;

    private final ControlBoard controlboard;
    public final SwerveDrive drive;

    public final IntakeSubsystem intake;
    public final IndexerSubsystem indexer;
    public final ShooterSubsystem shooter;

    public final AmpArmSubsystem ampArm;
    public final AmpIntakeSubsystem ampIntake;

    public final ClimberSubsystem climber;

    public RobotContainer() {
        if (RobotBase.isSimulation()) {
            DriverStationSim.setAllianceStationId(Math.random() > 0.5 ?
                    AllianceStationID.Blue1 : AllianceStationID.Red1);

            SimView.publish();
        }

        // Turn off joystick warnings in sim
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

        // Initialize Messenger
        String host = RobotBase.isSimulation() ? MESSENGER_HOST_SIM : MESSENGER_HOST_ROBOT;
        messenger = new MessengerClient(host, MESSENGER_PORT, MESSENGER_NAME);
        new FileSystemAPI(messenger, "RoboRIO", Filesystem.getOperatingDirectory());

        drive = new SwerveDrive(FieldInfo.CRESCENDO_2024, messenger);

        intake = new IntakeSubsystem();
        indexer = new IndexerSubsystem(intake);
        shooter = new ShooterSubsystem(drive, indexer);

        ampArm = new AmpArmSubsystem();
        ampIntake = new AmpIntakeSubsystem();

        climber = new ClimberSubsystem();

        // ControlBoard must be initialized last
        controlboard = new ControlBoard(this);

        // Register Named Commands for Auto
        NamedCommands.registerCommand("Example Named Command", new InstantCommand());

        // Create a chooser to select the autonomous
        autoSelector = new LoggedDashboardChooser<>("Auto Selection", AutoBuilder.buildAutoChooser());
        autoSelector.addDefaultOption("Drive forward", Commands.run(
                () -> drive.drive(new DriveRequest(
                        SwerveDrive.AUTO_PRIORITY,
                        new Translation2d(0.0, 0.5),
                        DriveRequestType.OpenLoopVoltage)),
                drive
        ).withTimeout(5));

        autoSelector.addOption("Example other auto", new PrintCommand("Example Auto"));

        FieldView.publish();
    }

    public Command getAutonomousCommand() {
        return autoSelector.get();
    }
}
