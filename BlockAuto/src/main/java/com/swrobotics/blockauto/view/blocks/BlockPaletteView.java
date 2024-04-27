package com.swrobotics.blockauto.view.blocks;

import com.swrobotics.blockauto.blocks.BlockCategory;
import com.swrobotics.blockauto.blocks.BlockDefinition;
import com.swrobotics.blockauto.blocks.BlockInstance;
import com.swrobotics.blockauto.blocks.BlockRegistry;
import com.swrobotics.blockauto.blocks.part.*;
import com.swrobotics.blockauto.view.View;
import imgui.ImGui;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class BlockPaletteView implements View {
    private BlockRegistry blockRegistry;
    private final Map<String, BlockInstance> previewInstances = new HashMap<>();

    public BlockPaletteView() {
        List<BlockCategory> categories = new ArrayList<>();

        List<BlockDefinition> category1 = new ArrayList<>();
        category1.add(new BlockDefinition("block1", List.of(new TextPart("Only text"))));
        category1.add(new BlockDefinition("block2", List.of(
                new TextPart("Double"),
                new DoubleParameterPart("double", 123.456))));
        category1.add(new BlockDefinition("block3", List.of(
                new TextPart("Enum"),
                new EnumParameterPart("enum", List.of("AAAAAA", "BBBBBB", "CCCCCC", "DDDDDD"), "AAAAAA"))));
        category1.add(new BlockDefinition("block4", List.of(
                new TextPart("Int"),
                new IntParameterPart("int", 123456))));
        category1.add(new BlockDefinition("block5", List.of(
                new TextPart("String"),
                new StringParameterPart("string", "hi i am text"))));
        categories.add(new BlockCategory("This category", category1));

        List<BlockDefinition> category2 = new ArrayList<>();
        category2.add(new BlockDefinition("block6", List.of(
                new TextPart("text on"),
                new TextPart("one line"),
                LineBreakPart.INSTANCE,
                new TextPart("text on another line!")
        )));
        category2.add(new BlockDefinition("block7", List.of(
                new TextPart("If has note:"),
                BlockStackParameterPart.INSTANCE,
                new TextPart("If no note:"),
                BlockStackParameterPart.INSTANCE
        )));
        categories.add(new BlockCategory("That category", category2));

        blockRegistry = new BlockRegistry(categories);


        for (BlockCategory category : blockRegistry.getCategories()) {
            for (BlockDefinition def : category.getBlocks()) {
                previewInstances.put(def.getName(), def.createDefaultInstance());
            }
        }
    }

    @Override
    public void process() {
        if (ImGui.begin("Block Palette")) {
            if (blockRegistry == null) {
                ImGui.textDisabled("No registry");
                ImGui.end();
                return;
            }

            for (BlockCategory category : blockRegistry.getCategories()) {
                if (ImGui.collapsingHeader(category.getName())) {
                    for (BlockDefinition blockDef : category.getBlocks()) {
                        ImGui.spacing();
                        BlockInstance inst = previewInstances.get(blockDef.getName());
                        inst.draw(null);
                    }
                    ImGui.spacing();
                }
            }
        }
        ImGui.end();
    }
}
