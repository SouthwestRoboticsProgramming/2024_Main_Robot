package com.swrobotics.shufflelog.tool.tetris;

public final class Mino {
    public final int offsetX;
    public final int offsetY;

    public Mino(int x, int y) {
        this.offsetX = x;
        this.offsetY = y;
    }

    public Mino rotate(Rotation rot) {
        return switch (rot) {
            case NONE -> this;
            case CW_90 -> new Mino(-offsetY, offsetX);
            case R_180 -> new Mino(-offsetX, -offsetY);
            case CCW_90 -> new Mino(offsetY, -offsetX);
        };
    }
}
