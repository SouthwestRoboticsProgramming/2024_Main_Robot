package com.swrobotics.blockauto.tool.field.path;

import com.swrobotics.messenger.client.MessageReader;

import java.util.BitSet;

public final class Grid2D {
    private int width;
    private int height;
    private BitSet data;

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public boolean get(int x, int y) {
        return data.get(x + y * width);
    }

    public void readContent(MessageReader reader) {
        width = reader.readInt();
        height = reader.readInt();
        int len = reader.readInt();
        long[] l = new long[len];
        for (int i = 0; i < len; i++) {
            l[i] = reader.readLong();
        }
        data = BitSet.valueOf(l);
    }
}
