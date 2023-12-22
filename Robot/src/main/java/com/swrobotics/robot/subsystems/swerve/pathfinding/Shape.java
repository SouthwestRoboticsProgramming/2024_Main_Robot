package com.swrobotics.robot.subsystems.swerve.pathfinding;

import com.swrobotics.messenger.client.MessageBuilder;

public interface Shape {
    byte CIRCLE = 0;
    byte RECTANGLE = 1;

    void write(MessageBuilder builder);
}
