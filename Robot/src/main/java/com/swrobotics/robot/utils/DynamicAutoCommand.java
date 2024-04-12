package com.swrobotics.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;

public class DynamicAutoCommand extends Command {
    private final List<Command> commands;
    private int i = 0;
    private Command currentCommand;

    public DynamicAutoCommand(List<Command> commands) {
        this.commands = commands;
        currentCommand = commands.get(0);
    }

    @Override
    public void initialize() {
        if (currentCommand != null) {
            currentCommand.cancel();
        }
        i = 0;
        currentCommand = commands.get(0);
        currentCommand.schedule();
        System.out.println(commands);
    }

    @Override
    public void execute() {
        if (!currentCommand.isFinished() && currentCommand.isScheduled()) {
            return;
        } else {
            i++;
            try {
                currentCommand = commands.get(i);
                currentCommand.schedule();
            } catch (IndexOutOfBoundsException e) {
                end(false);
            }
            
        }
    }

    @Override
    public boolean isFinished() {
        return i == commands.size() && currentCommand.isFinished();
    }
}
