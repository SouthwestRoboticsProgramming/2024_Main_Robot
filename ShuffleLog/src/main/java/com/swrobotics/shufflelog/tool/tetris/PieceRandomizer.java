package com.swrobotics.shufflelog.tool.tetris;

import java.util.ArrayList;
import java.util.List;

public final class PieceRandomizer {
    private final List<Integer> bag = new ArrayList<>(TetrisTool.SHAPES.length);

    public PieceRandomizer() {
        refill();
    }

    public int getNext() {
        int idx = (int) (Math.random() * bag.size());
        int shape = bag.remove(idx);
        if (bag.isEmpty())
            refill();
        return shape;
    }

    private void refill() {
        for (int i = 0; i < TetrisTool.SHAPES.length; i++) {
            bag.add(i);
        }
    }
}
