package com.swrobotics.blockauto.tool.tetris;

public final class TetrominoShape {
    public final TileState state;
    public final Mino[] minos;

    public TetrominoShape(TileState state, Mino... minos) {
        this.state = state;
        this.minos = minos;
    }
}
