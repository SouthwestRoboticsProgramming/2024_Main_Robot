package com.swrobotics.blockauto.blocks;

import com.swrobotics.blockauto.blocks.part.BlockPart;
import com.swrobotics.blockauto.blocks.part.LineBreakPart;
import com.swrobotics.blockauto.blocks.part.ParameterPart;
import com.swrobotics.blockauto.blocks.part.TextPart;
import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiDragDropFlags;

import java.util.List;

public final class BlockInstance {
    private final int BORDER_COLOR = ImGui.colorConvertFloat4ToU32(0.5f, 0.5f, 0.5f, 1.0f);

    private final BlockDefinition definition;
    private final Object[] parameterVals;

    // Parameter values are expected to be immutable
    public BlockInstance(BlockDefinition definition, Object[] parameterVals) {
        this.definition = definition;
        this.parameterVals = parameterVals;
    }

    public void draw(BlockStack stack) {
        int parameterIdx = 0;
        List<BlockPart> parts = definition.getParts();

        ImGui.beginGroup();
        boolean onSameLine = false;
        Boolean shouldAlignText = null;
        for (int i = 0; i < parts.size(); i++) {
            BlockPart part = parts.get(i);
            if (!part.isInline()) {
                onSameLine = false;
                shouldAlignText = null;

                if (part == LineBreakPart.INSTANCE)
                    continue;
            }

            if (shouldAlignText == null) {
                for (int j = i; j < parts.size(); j++) {
                    BlockPart part2 = parts.get(j);
                    if (!part.isInline())
                        break;

                    if (part2.isFramed()) {
                        shouldAlignText = true;
                        break;
                    }
                }
                if (shouldAlignText == null)
                    shouldAlignText = false;
            }

            if (onSameLine)
                ImGui.sameLine();
            onSameLine = part.isInline();

            if (part instanceof ParameterPart p) {
                parameterVals[parameterIdx] = p.edit(parameterVals[parameterIdx]);
                parameterIdx++;
                continue;
            }
            if (part instanceof TextPart p) {
                if (shouldAlignText)
                    ImGui.alignTextToFramePadding();
                p.draw();
            }
        }

        ImGui.endGroup();
        ImVec2 rectMin = ImGui.getItemRectMin();
        ImVec2 rectMax = ImGui.getItemRectMax();
        float padding = ImGui.getStyle().getItemSpacingY() / 2;
        ImGui.getWindowDrawList().addRect(
                rectMin.x - padding,
                rectMin.y - padding,
                rectMax.x + padding,
                rectMax.y + padding,
                BORDER_COLOR
        );

        if (ImGui.beginDragDropSource(ImGuiDragDropFlags.SourceAllowNullID)) {
            ImGui.setDragDropPayload(BlockStack.DRAG_DROP_TYPE, new BlockDragInfo(stack, this));
            draw(null);
            ImGui.endDragDropSource();
        }
    }

    public BlockDefinition getDefinition() {
        return definition;
    }

    public Object[] getParameterVals() {
        return parameterVals;
    }

    public BlockInstance createCopy() {
        Object[] paramsCopy = new Object[parameterVals.length];
        int i = 0;
        for (BlockPart part : definition.getParts()) {
            if (part instanceof ParameterPart p) {
                paramsCopy[i] = p.copyValue(parameterVals[i]);
                i++;
            }
        }

        return new BlockInstance(definition, paramsCopy);
    }
}
