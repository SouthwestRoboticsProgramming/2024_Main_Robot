package com.swrobotics.shufflelog.tool.field.path.shape;

import com.swrobotics.messenger.client.MessageBuilder;
import com.swrobotics.messenger.client.MessageReader;

import java.util.UUID;

public abstract class Shape {
    public static final byte CIRCLE = 0;
    public static final byte RECTANGLE = 1;

    private final UUID id;

    public Shape(UUID id) {
        this.id = id;
    }

    public UUID getId() {
        return id;
    }

    protected abstract void readContent(MessageReader reader);

    public static Shape read(MessageReader reader) {
        long idMsb = reader.readLong();
        long idLsb = reader.readLong();
        UUID id = new UUID(idMsb, idLsb);

        byte type = reader.readByte();
        Shape shape;
        switch (type) {
            case CIRCLE:
                shape = new Circle(id);
                break;
            case RECTANGLE:
                shape = new Rectangle(id);
                break;
            default:
                throw new RuntimeException("Unknown type id: " + type);
        }
        shape.readContent(reader);

        return shape;
    }

    public void write(MessageBuilder builder) {
        builder.addLong(id.getMostSignificantBits());
        builder.addLong(id.getLeastSignificantBits());
    }
}
