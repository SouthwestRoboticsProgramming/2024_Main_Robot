package com.swrobotics.blockauto.blocks.part;

import com.swrobotics.blockauto.blocks.BlockStack;
import imgui.ImGui;

import java.util.ArrayList;

public final class BlockStackParameterPart implements ParameterPart {
    public static final BlockStackParameterPart INSTANCE = new BlockStackParameterPart();

    private BlockStackParameterPart() {}

    @Override
    public Object getDefaultValue() {
        return new BlockStack(new ArrayList<>());
    }

    @Override
    public Object edit(Object value) {
        ImGui.indent();
        ((BlockStack) value).draw();
        ImGui.unindent();
        return value; // Mutates in-place
    }

    @Override
    public boolean isFramed() {
        return false;
    }

    @Override
    public boolean isInline() {
        return false;
    }

    @Override
    public Object copyValue(Object value) {
        return ((BlockStack) value).createCopy();
    }
}
