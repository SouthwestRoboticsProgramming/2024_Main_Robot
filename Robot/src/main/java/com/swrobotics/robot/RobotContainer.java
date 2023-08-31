package com.swrobotics.robot;

import com.swrobotics.messenger.client.MessengerClient;
import com.swrobotics.taskmanager.filesystem.FileSystemAPI;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Supplier;

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
    private final SendableChooser<Supplier<Command>> autoSelector;

    public final MessengerClient messenger;

    public RobotContainer() {
        // Turn off joystick warnings in sim
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

        // Initialize Messenger
        String host = RobotBase.isSimulation() ? MESSENGER_HOST_SIM : MESSENGER_HOST_ROBOT;
        messenger = new MessengerClient(host, MESSENGER_PORT, MESSENGER_NAME);
        new FileSystemAPI(messenger, "RoboRIO", Filesystem.getOperatingDirectory());

        // Autos that don't do anything
        Command blankAuto = new InstantCommand();

        // Create a chooser to select the autonomous
        autoSelector = new SendableChooser<>();
        autoSelector.addOption("No Auto", () -> blankAuto);
        SmartDashboard.putData("Auto", autoSelector);
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected().get();
    }
}
