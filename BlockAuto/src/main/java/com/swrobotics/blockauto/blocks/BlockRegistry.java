package com.swrobotics.blockauto.blocks;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class BlockRegistry {
    private final List<BlockCategory> categories;
    private final Map<String, BlockDefinition> blocksByName;

    public BlockRegistry(List<BlockCategory> categories) {
        this.categories = categories;

        blocksByName = new HashMap<>();
        for (BlockCategory category : categories) {
            for (BlockDefinition block : category.getBlocks()) {
                blocksByName.put(block.getName(), block);
            }
        }
    }

    public List<BlockCategory> getCategories() {
        return categories;
    }
}
