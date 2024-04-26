package com.swrobotics.blockauto.tool.tetris;

public enum TileState {
    EMPTY(0xFF000000),
    LIGHTBLUE(0xFF00FFFF),
    BLUE(0xFF0000FF),
    ORANGE(0xFFFF7F00),
    YELLOW(0xFFFFFF00),
    GREEN(0xFF00FF00),
    PURPLE(0xFF800080),
    RED(0xFFFF0000);

    public final int color;

    TileState(int color) {
        this.color = color;
    }
}
