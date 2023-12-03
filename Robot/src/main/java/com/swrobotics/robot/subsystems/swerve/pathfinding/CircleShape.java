package com.swrobotics.robot.subsystems.swerve.pathfinding;

import com.swrobotics.messenger.client.MessageBuilder;

public final class CircleShape implements Shape {
    public final double x, y;
    public final double radius;

    public CircleShape(double x, double y, double radius) {
        this.x = x;
        this.y = y;
        this.radius = radius;
    }

    @Override
    public void write(MessageBuilder builder) {
        builder.addByte(CIRCLE);
        builder.addDouble(x);
        builder.addDouble(y);
        builder.addDouble(radius);
    }

    @Override
    public String toString() {
        return "CircleObstacle{" +
                "x=" + x +
                ", y=" + y +
                ", radius=" + radius +
                '}';
    }
}
