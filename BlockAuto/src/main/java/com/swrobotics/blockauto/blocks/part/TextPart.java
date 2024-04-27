package com.swrobotics.blockauto.blocks.part;

import imgui.ImGui;

public final class TextPart implements BlockPart {
    private final String text;

    public TextPart(String text) {
        this.text = text;
    }

    public void draw() {
        ImGui.textUnformatted(text);
    }
}
