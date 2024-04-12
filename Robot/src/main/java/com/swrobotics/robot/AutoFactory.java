package com.swrobotics.robot;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.swrobotics.robot.commands.RobotCommands;
import com.swrobotics.robot.utils.DynamicAutoCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoFactory {
    private final RobotContainer robot;
    private final ArrayList<Command> sequence;

    public AutoFactory(RobotContainer robot) {
        this.robot = robot;
        sequence = new ArrayList<Command>();
    }

    public AutoFactory goToNote(Notes note) {
        sequence.add(note.getPath().grabNote);
        sequence.add(note.getPath().scoreNote);
        return this;
    }

    public AutoFactory shoot(boolean waitForNote) {
        sequence.add(RobotCommands.aimAndShoot(robot, waitForNote));
        return this;
    }

    public AutoFactory fling() {
        sequence.add(RobotCommands.flingNote(robot));
        return this;
    }

    public Command build() {
        // return new DynamicAutoCommand(sequence);
        Command[] commands = new Command[sequence.size()];
        return new SequentialCommandGroup(sequence.toArray(commands));
    }


    protected record NotePath(Command grabNote, Command scoreNote) {}

    public enum Notes {
        SOURCE_WALL(new NotePath(ppPath("Source Center A Grab"), ppPath("Source Center A Score")));//,
        // SOURCE_2,
        // CENTER,
        // AMP_2,
        // AMP_WALL;

        private final NotePath path;
        private Notes(NotePath path) {
            this.path = path;
        }

        public NotePath getPath() {
            return path;
        }
    }

    protected static Command ppPath(String name) {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(name));
    }

    protected static Command choreoPath(String name) {
        return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(name));
    }
}
