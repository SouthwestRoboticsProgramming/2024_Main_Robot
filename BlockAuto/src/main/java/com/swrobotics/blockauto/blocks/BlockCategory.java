package com.swrobotics.blockauto.blocks;

import java.util.List;

public final class BlockCategory {
    private final String name;
    private final List<BlockDefinition> blocks;

    public BlockCategory(String name, List<BlockDefinition> blocks) {
        this.name = name;
        this.blocks = blocks;
    }

    public String getName() {
        return name;
    }

    public List<BlockDefinition> getBlocks() {
        return blocks;
    }
}
