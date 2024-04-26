package com.swrobotics.blockauto.tool.field.path.shape;

import com.swrobotics.messenger.client.MessageBuilder;
import com.swrobotics.messenger.client.MessageReader;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;

import java.util.UUID;

public final class Rectangle extends Shape {
    public final ImDouble x;
    public final ImDouble y;
    public final ImDouble width;
    public final ImDouble height;
    public final ImDouble rotation;
    public final ImBoolean inverted;

    public Rectangle(UUID id, String name) {
        super(id, name);

        x = new ImDouble();
        y = new ImDouble();
        width = new ImDouble();
        height = new ImDouble();
        rotation = new ImDouble();
        this.inverted = new ImBoolean();
    }

    @Override
    protected void readContent(MessageReader reader) {
        x.set(reader.readDouble());
        y.set(reader.readDouble());
        width.set(reader.readDouble());
        height.set(reader.readDouble());
        rotation.set(Math.toDegrees(reader.readDouble()));
        inverted.set(reader.readBoolean());
    }

    @Override
    public void write(MessageBuilder builder) {
        super.write(builder);
        builder.addByte(RECTANGLE);
        builder.addDouble(x.get());
        builder.addDouble(y.get());
        builder.addDouble(width.get());
        builder.addDouble(height.get());
        builder.addDouble(Math.toRadians(rotation.get()));
        builder.addBoolean(inverted.get());
    }
}
