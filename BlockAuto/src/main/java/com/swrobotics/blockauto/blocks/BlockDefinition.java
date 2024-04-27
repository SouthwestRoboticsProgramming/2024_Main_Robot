package com.swrobotics.blockauto.blocks;

import com.swrobotics.blockauto.blocks.part.BlockPart;
import com.swrobotics.blockauto.blocks.part.ParameterPart;

import java.util.ArrayList;
import java.util.List;

public final class BlockDefinition {
    private final String name;
    private final List<BlockPart> parts;

    public BlockDefinition(String name, List<BlockPart> parts) {
        this.name = name;
        this.parts = parts;
    }

    public BlockInstance createDefaultInstance() {
        List<Object> paramVals = new ArrayList<>();
        for (BlockPart part : parts) {
            if (part instanceof ParameterPart p) {
                paramVals.add(p.getDefaultValue());
            }
        }
        return new BlockInstance(this, paramVals.toArray(new Object[0]));
    }

    public String getName() {
        return name;
    }

    public List<BlockPart> getParts() {
        return parts;
    }
}
