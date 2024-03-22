package com.swrobotics.robot;

import com.swrobotics.lib.ThreadUtils;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.Settings;
import com.swrobotics.robot.config.Settings.Mode;
import com.swrobotics.robot.config.Settings.RobotType;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.BiConsumer;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private final Timer autonomousTimer = new Timer();
    private double autonomousDelay;
    private boolean hasScheduledAuto;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {

        // Record metadata so that the logs have more to work off of
        // logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        // logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        // logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        // logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        // logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        // switch (BuildConstants.DIRTY) {
        //     case 0:
        //         logger.recordMetadata("GitDirty", "All changes committed");
        //         break;
        //     case 1:
        //         logger.recordMetadata("GitDirty", "Uncomitted changes");
        //         break;
        //     default:
        //         logger.recordMetadata("GitDirty", "Unknown");
        //         break;
        // }

        // Set up data receivers & replay source
        switch (Settings.getMode()) {
            case REAL:
                String folder = Settings.logFolders.get(Settings.robot);
                if (folder != null) {
                    Logger.addDataReceiver(new WPILOGWriter(folder));
                }
                Logger.addDataReceiver(new NT4Publisher());
                if (Settings.robot == RobotType.COMPETITION) {
                    LoggedPowerDistribution.getInstance(
                            IOAllocation.CAN.PDP.id(), ModuleType.kRev);
                }
                break;

            case SIMULATION:
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                String path = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(path));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
                break;
        }

        // Start AdvantageKit logger
        setUseTiming(Settings.getMode() != Mode.REPLAY);
        Logger.start();

        // Create a RobotContainer to manage our subsystems and our buttons
        robotContainer = new RobotContainer();

        // Log active commands
        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction =
                (Command command, Boolean active) -> {
                    String name = command.getName();
                    int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
                    commandCounts.put(name, count);
                    // Logger.recordOutput(
                    //                 "CommandsUnique/"
                    //                         + name
                    //                         + "_"
                    //                         + Integer.toHexString(command.hashCode()),
                    //                 active);
                    Logger.recordOutput("CommandsAll/" + name, count > 0);
                };
        CommandScheduler.getInstance()
                .onCommandInitialize(
                        (Command command) -> {
                            logCommandFunction.accept(command, true);
                        });
        CommandScheduler.getInstance()
                .onCommandFinish(
                        (Command command) -> {
                            logCommandFunction.accept(command, false);
                        });
        CommandScheduler.getInstance()
                .onCommandInterrupt(
                        (Command command) -> {
                            logCommandFunction.accept(command, false);
                        });
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);

        robotContainer.messenger.readMessages();
        ThreadUtils.runMainThreadOperations();
        CommandScheduler.getInstance().run(); // Leave this alone
    }

    @Override
    public void autonomousInit() {
        // If an autonomous command has already be set, reset it
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            System.out.println("Canceled the current auto command");
        }

        // Get autonomous from selector
        autonomousCommand = robotContainer.getAutonomousCommand();

        autonomousDelay = robotContainer.getAutoDelay();
        autonomousTimer.restart();
        hasScheduledAuto = false;
    }

    @Override
    public void autonomousPeriodic() {
        // Manually time auto delay since using sequential group causes crash
        // when running the same auto twice
        if (!hasScheduledAuto && autonomousCommand != null && autonomousTimer.hasElapsed(autonomousDelay)) {
            autonomousCommand.schedule();
            hasScheduledAuto = true;
        }
    }

    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void disabledInit() {
        robotContainer.disabledInit();
    }

    @Override
    public void disabledExit() {
        robotContainer.disabledExit();
    }
}
