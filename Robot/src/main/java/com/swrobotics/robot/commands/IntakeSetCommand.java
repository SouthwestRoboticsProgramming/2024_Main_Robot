package com.swrobotics.robot.commands;

import com.swrobotics.robot.subsystems.speaker.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public final class IntakeSetCommand extends Command {
    private final IntakeSubsystem intake;
    private final IntakeSubsystem.State state;

    public IntakeSetCommand(IntakeSubsystem intake, IntakeSubsystem.State state) {
        this.intake = intake;
        this.state = state;
    }

    @Override
    public void execute() {
        intake.set(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
