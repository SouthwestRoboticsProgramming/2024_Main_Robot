package com.swrobotics.shufflelog.tool.tetris;

import processing.core.PGraphics;

import java.util.function.BiConsumer;

public final class TetrominoContext {
    public int shapeIdx;
    public int x, y;
    public Rotation rot;

    public TetrominoContext() {
        rot = Rotation.NONE;
    }

    public TetrominoShape getShape() {
        return TetrisTool.SHAPES[shapeIdx];
    }

    public void forEachMino(BiConsumer<Integer, Integer> posFn) {
        for (Mino mino : getShape().minos) {
            Mino rotated = mino.rotate(rot);
            posFn.accept(x + rotated.offsetX, y + rotated.offsetY);
        }
    }

    public void draw(PGraphics g, boolean ghost) {
        int col = getShape().state.color;
        if (ghost) {
            col = g.color(g.red(col), g.green(col), g.blue(col), 128);
        }

        final int col_ = col;
        forEachMino((x, y) -> {
            TetrisTool.drawMino(g, x, y, col_);
        });
    }
}
