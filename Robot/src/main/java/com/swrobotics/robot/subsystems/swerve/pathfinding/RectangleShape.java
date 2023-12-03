package com.swrobotics.robot.subsystems.swerve.pathfinding;

import com.swrobotics.messenger.client.MessageBuilder;
import edu.wpi.first.math.geometry.Rotation2d;

public final class RectangleShape implements Shape {
    public final double x, y;
    public final double width, height;
    public final Rotation2d rotation;
    public final boolean inverted;

    public RectangleShape(double x, double y, double width, double height, Rotation2d rotation, boolean inverted) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.rotation = rotation;
        this.inverted = inverted;
    }

    @Override
    public void write(MessageBuilder builder) {
        builder.addByte(RECTANGLE);
        builder.addDouble(x);
        builder.addDouble(y);
        builder.addDouble(width);
        builder.addDouble(height);
        builder.addDouble(rotation.getRadians());
        builder.addBoolean(inverted);
    }

    @Override
    public String toString() {
        return "RectangleObstacle{" +
                "x=" + x +
                ", y=" + y +
                ", width=" + width +
                ", height=" + height +
                ", rotation=" + rotation +
                ", inverted=" + inverted +
                '}';
    }
}
