package com.swrobotics.blockauto.blocks.part;

public final class LineBreakPart implements BlockPart {
    public static final LineBreakPart INSTANCE = new LineBreakPart();

    private LineBreakPart() {}

    @Override
    public boolean isInline() {
        return false;
    }
}
