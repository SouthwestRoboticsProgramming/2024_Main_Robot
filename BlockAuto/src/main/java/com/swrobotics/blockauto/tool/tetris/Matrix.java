package com.swrobotics.blockauto.tool.tetris;

import processing.core.PGraphics;

public final class Matrix {
    public static final int matrixWidth = 10;
    public static final int matrixHeight = 20;

    private final TileState[][] tiles = new TileState[matrixWidth][matrixHeight];

    public Matrix() {
        for (int x = 0; x < matrixWidth; x++) {
            for (int y = 0; y < matrixHeight; y++) {
                tiles[x][y] = TileState.EMPTY;
            }
        }
    }

    private boolean inBounds(int x, int y) {
        return x >= 0 && x < matrixWidth && y >= 0 && y < matrixHeight;
    }

    private void set(int x, int y, TileState state) {
        if (inBounds(x, y)) {
            tiles[x][y] = state;
        }
    }

    private TileState get(int x, int y) {
        return tiles[x][y];
    }

    public int plot(TetrominoContext ctx) {
        int[] minY = {Integer.MAX_VALUE};
        int[] maxY = {0};
        ctx.forEachMino((mx, my) -> {
            set(mx, my, ctx.getShape().state);
            minY[0] = Math.min(minY[0], my);
            maxY[0] = Math.max(maxY[0], my);
        });

        int linesCleared = 0;

        // Check for completed rows
        int count = maxY[0] - minY[0] + 1;
        int y = maxY[0];
        row: for (int i = 0; i < count; i++) {
            for (int x = 0; x < matrixWidth; x++) {
                if (get(x, y) == TileState.EMPTY) {
                    y--;
                    continue row;
                }
            }

            linesCleared++;

            // Move above down
            for (int row = y; row > 0; row--) {
                for (int x = 0; x < matrixWidth; x++) {
                    set(x, row, get(x, row - 1));
                }
            }

            // Clear top row
            for (int x = 0; x < matrixWidth; x++) {
                set(x, 0, TileState.EMPTY);
            }
        }

        return linesCleared;
    }

    public boolean collides(TetrominoContext ctx) {
        boolean[] hit = {false};
        ctx.forEachMino((mx, my) -> {
            hit[0] |= !inBounds(mx, my) || get(mx, my) != TileState.EMPTY;
        });
        return hit[0];
    }

    public void draw(PGraphics g) {
        for (int x = 0; x < matrixWidth; x++) {
            for (int y = 0; y < matrixHeight; y++) {
                TetrisTool.drawMino(g, x, y, tiles[x][y].color);
            }
        }
    }

    public void clear() {
        for (int y = 0; y < matrixHeight; y++) {
            for (int x = 0; x < matrixWidth; x++) {
                set(x, y, TileState.EMPTY);
            }
        }
    }
}
