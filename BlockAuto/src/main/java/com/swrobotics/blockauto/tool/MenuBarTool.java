package com.swrobotics.blockauto.tool;

import imgui.ImGui;

public final class MenuBarTool implements Tool {
    public MenuBarTool() {
    }

    @Override
    public void process() {
        if (ImGui.beginMainMenuBar()) {
            if (ImGui.beginMenu("This might be a menu someday")) {
                ImGui.endMenu();
            }

            ImGui.endMainMenuBar();
        }
    }
}
