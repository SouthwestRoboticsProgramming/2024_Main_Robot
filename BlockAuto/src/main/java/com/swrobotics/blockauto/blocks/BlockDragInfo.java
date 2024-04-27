package com.swrobotics.blockauto.blocks;

public final class BlockDragInfo {
    private final BlockStack stack;
    private final BlockInstance block;

    // Stack is nullable
    public BlockDragInfo(BlockStack stack, BlockInstance block) {
        this.stack = stack;
        this.block = block;
    }

    public BlockStack getStack() {
        return stack;
    }

    public BlockInstance getBlock() {
        return block;
    }
}
