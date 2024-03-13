package com.swrobotics.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.swrobotics.lib.field.FieldInfo;
import com.swrobotics.messenger.client.MessengerClient;
import com.swrobotics.robot.commands.RobotCommands;
import com.swrobotics.robot.commands.IntakeSetCommand;
import com.swrobotics.robot.commands.PlaySongCommand;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.control.ControlBoard;
import com.swrobotics.robot.logging.FieldView;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.subsystems.amp.AmpArm2Subsystem;
import com.swrobotics.robot.subsystems.amp.AmpArmSubsystem;
import com.swrobotics.robot.subsystems.amp.AmpIntakeSubsystem;
import com.swrobotics.robot.subsystems.climber.ClimberSubsystem;
import com.swrobotics.robot.subsystems.lights.LightsSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

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
    private final LoggedDashboardChooser<String> musicSelector;
    private final LoggedDashboardChooser<Double> autoDelaySelector;

    public final MessengerClient messenger;
    private final ControlBoard controlboard;

    public final PowerDistribution pdp;

    // Mechanical
    public final SwerveDrive drive;
    public final IntakeSubsystem intake;
    public final IndexerSubsystem indexer;
    public final ShooterSubsystem shooter;
//    public final AmpArmSubsystem ampArm;
    public final AmpArm2Subsystem ampArm2;
    public final AmpIntakeSubsystem ampIntake;
    public final ClimberSubsystem climber;

    // Fun
    public final LightsSubsystem lights;
    public final MusicSubsystem music;

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

        pdp = new PowerDistribution(IOAllocation.CAN.PDP.id(), PowerDistribution.ModuleType.kRev);

        drive = new SwerveDrive(FieldInfo.CRESCENDO_2024, messenger);

        intake = new IntakeSubsystem(pdp);
        indexer = new IndexerSubsystem(intake);
        shooter = new ShooterSubsystem(drive, indexer);

//        ampArm = new AmpArmSubsystem();
        ampArm2 = new AmpArm2Subsystem();
        ampIntake = new AmpIntakeSubsystem();

        climber = new ClimberSubsystem();

        // ControlBoard must be initialized last
        lights = new LightsSubsystem(this);
        music = new MusicSubsystem(this);
        controlboard = new ControlBoard(this);

        // Register Named Commands for Auto
        NamedCommands.registerCommand("Intake On", new IntakeSetCommand(intake, IntakeSubsystem.State.INTAKE));
        NamedCommands.registerCommand("Intake Off", new IntakeSetCommand(intake, IntakeSubsystem.State.OFF));
        NamedCommands.registerCommand("Shoot", RobotCommands.aimAndShoot(this, false));
        NamedCommands.registerCommand("Wait and Shoot", RobotCommands.aimAndShoot(this, true)); // Waits for indexer to have note

        // Create a chooser to select the autonomous
        List<AutoEntry> autos = buildPathPlannerAutos();
        autos.add(new AutoEntry("Drive backward", Commands.run(
                () -> drive.drive(new DriveRequest(
                        SwerveDrive.AUTO_PRIORITY,
                        new Translation2d(-0.5, 0),
                        DriveRequestType.OpenLoopVoltage)),
                drive
        ).withTimeout(5)));
        autos.sort(Comparator.comparing(AutoEntry::name, String.CASE_INSENSITIVE_ORDER));

        SendableChooser<Command> autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Just Shoot", RobotCommands.aimAndShoot(this, false));
        autoChooser.addOption("None", Commands.none());
        for (AutoEntry auto : autos)
            autoChooser.addOption(auto.name(), auto.cmd());
        autoSelector = new LoggedDashboardChooser<>("Auto Selection", autoChooser);

        FieldView.publish();

        char sep = File.separatorChar;
        String whichSong = Math.random() > 0.9 ? "caramell-bert-expanded.chrp" : "xp.chrp";
        CommandScheduler.getInstance().schedule(musicCommand = Commands.waitSeconds(5)
                .andThen(new PlaySongCommand(music, "music" + sep + whichSong)));

        SendableChooser<String> musicChooser = new SendableChooser<>();
        for (String song : MusicSubsystem.getAvailableSongs()) {
            int lastIdx = song.lastIndexOf(File.separatorChar);

            String option = song.substring(lastIdx + 1);
            musicChooser.addOption(option, song);
        }
        musicChooser.setDefaultOption("None", "None");
        musicSelector = new LoggedDashboardChooser<>("Victory Music Selection", musicChooser);

        // Rumble the controllers when we pick up a piece
        new Trigger(() -> indexer.hasPiece()).onTrue(
            Commands.run(() -> controlboard.setPieceRumble(true))
            .withTimeout(0.2)
            .andThen(
                Commands.runOnce(() -> controlboard.setPieceRumble(false))
            )
        );


        SendableChooser<Double> autoDelay = new SendableChooser<>();
        autoDelay.setDefaultOption("None", 0.0);
        for (int i = 0; i < 10; i++) {
            double time = i / 2.0 + 0.5;
            autoDelay.addOption(time + " seconds", time);
        }
        autoDelaySelector = new LoggedDashboardChooser<>("Auto Delay", autoDelay);
    }

    private static final record AutoEntry(String name, Command cmd) {}

    private static List<AutoEntry> buildPathPlannerAutos() {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException(
                    "AutoBuilder was not configured before attempting to build an auto chooser");
        }

        List<String> autoNames = AutoBuilder.getAllAutoNames();
        autoNames.sort(String.CASE_INSENSITIVE_ORDER);

        List<PathPlannerAuto> options = new ArrayList<>();
        for (String autoName : autoNames) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);

            options.add(auto);
        }

        List<AutoEntry> entries = new ArrayList<>();
        for (PathPlannerAuto auto : options)
            entries.add(new AutoEntry(auto.getName(), auto));

        return entries;
    }

    private boolean hasDoneFirstInit = false;
    private Command musicCommand;
    public void disabledInit() {
        lights.disabledInit();

        String song = musicSelector.get();
        if (hasDoneFirstInit && !song.equals("None"))
            CommandScheduler.getInstance().schedule(musicCommand = new PlaySongCommand(music, song));
        hasDoneFirstInit = true;
    }

    public void disabledExit() {
        if (musicCommand != null)
            CommandScheduler.getInstance().cancel(musicCommand);
    }

    public double getAutoDelay() {
        return autoDelaySelector.get();
    }

    public Command getAutonomousCommand() {
        return autoSelector.get();
    }
}
