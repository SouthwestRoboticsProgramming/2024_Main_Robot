package com.swrobotics.blockauto.view;

import imgui.ImGui;

public final class MenuBarView implements View {
    public MenuBarView() {
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
