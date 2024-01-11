package com.swrobotics.shufflelog.tool.field.path.shape;

import com.swrobotics.messenger.client.MessageBuilder;
import com.swrobotics.messenger.client.MessageReader;
import imgui.type.ImString;

import java.nio.charset.StandardCharsets;
import java.util.UUID;

public abstract class Shape {
    public static final byte CIRCLE = 0;
    public static final byte RECTANGLE = 1;

    private final UUID id;
    public final ImString name;

    public Shape(UUID id, String name) {
        this.id = id;
        if (name == null) {
            this.name = null;
        } else {
            this.name = new ImString(128);
            this.name.set(name);
        }
    }

    public UUID getId() {
        return id;
    }

    protected abstract void readContent(MessageReader reader);

    public static Shape read(MessageReader reader, boolean hasUuid, boolean hasName) {
        UUID id = null;
        if (hasUuid) {
            long idMsb = reader.readLong();
            long idLsb = reader.readLong();
            id = new UUID(idMsb, idLsb);
        }
        String name = null;
        if (hasName) {
            int len = reader.readShort();
            byte[] data = reader.readRaw(len);
            name = new String(data, StandardCharsets.UTF_8);
        }

        byte type = reader.readByte();
        Shape shape;
        switch (type) {
            case CIRCLE:
                shape = new Circle(id, name);
                break;
            case RECTANGLE:
                shape = new Rectangle(id, name);
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

        byte[] nameUtf8 = name.get().getBytes(StandardCharsets.UTF_8);
        builder.addShort((short) nameUtf8.length);
        builder.addRaw(nameUtf8);
    }
}
