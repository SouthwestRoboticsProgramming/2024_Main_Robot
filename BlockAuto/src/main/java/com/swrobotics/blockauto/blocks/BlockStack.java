package com.swrobotics.blockauto.blocks;

import imgui.ImGui;
import imgui.flag.ImGuiDragDropFlags;

import java.util.ArrayList;
import java.util.List;

public final class BlockStack {
    public static final String DRAG_DROP_TYPE = "Auto_Block";

    private final List<BlockInstance> blocks;

    public BlockStack(List<BlockInstance> blocks) {
        this.blocks = blocks;
    }

    public BlockStack createCopy() {
        List<BlockInstance> copy = new ArrayList<>();
        for (BlockInstance inst : blocks) {
            copy.add(inst.createCopy());
        }
        return new BlockStack(copy);
    }

    private void acceptBlock(int insertionIdx) {
        if (ImGui.beginDragDropTarget()) {
            BlockDragInfo droppedBlock = ImGui.acceptDragDropPayload(DRAG_DROP_TYPE);
            if (droppedBlock != null) {
                BlockStack srcStack = droppedBlock.getStack();
                if (srcStack != null) {
                    if (srcStack == this && srcStack.blocks.indexOf(droppedBlock.getBlock()) < insertionIdx)
                        insertionIdx--;

                    srcStack.blocks.remove(droppedBlock.getBlock());
                }

                blocks.add(insertionIdx, droppedBlock.getBlock().createCopy());
            }
            ImGui.endDragDropTarget();
        }
    }

    public void draw() {
        int idx = 0;
        for (BlockInstance block : new ArrayList<>(blocks)) {
            ImGui.beginGroup();
            ImGui.pushID(idx);
            block.draw(this);
            ImGui.popID();
            ImGui.endGroup();

            acceptBlock(idx++);
        }

        ImGui.textUnformatted("Add blocks here...");
        acceptBlock(idx);
    }
}
