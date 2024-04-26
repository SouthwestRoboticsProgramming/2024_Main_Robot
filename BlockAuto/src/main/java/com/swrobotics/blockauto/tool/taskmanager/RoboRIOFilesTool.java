package com.swrobotics.blockauto.tool.taskmanager;

import com.swrobotics.blockauto.BlockAuto;
import com.swrobotics.blockauto.tool.Tool;
import com.swrobotics.blockauto.tool.taskmanager.file.RemoteFileView;

import imgui.ImGui;

public final class RoboRIOFilesTool implements Tool {
    private final RemoteFileView fileView;

    public RoboRIOFilesTool(BlockAuto log) {
        fileView = new RemoteFileView(log, "RoboRIO");
    }

    @Override
    public void process() {
        if (ImGui.begin("RoboRIO Files")) {
            fileView.process();
        }
        ImGui.end();
    }
}
