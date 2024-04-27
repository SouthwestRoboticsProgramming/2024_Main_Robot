package com.swrobotics.blockauto.view.blocks;

import com.swrobotics.blockauto.blocks.BlockStack;
import com.swrobotics.blockauto.view.View;
import imgui.ImGui;

import java.util.ArrayList;

public final class BlockCanvasView implements View {
    private final BlockStack stack = new BlockStack(new ArrayList<>());

    @Override
    public void process() {
        if (ImGui.begin("Block Canvas")) {
            stack.draw();
        }
        ImGui.end();
    }
}
