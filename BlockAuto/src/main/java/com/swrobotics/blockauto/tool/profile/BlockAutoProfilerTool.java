package com.swrobotics.blockauto.tool.profile;

import com.swrobotics.blockauto.BlockAuto;
import com.swrobotics.blockauto.profiler.MemoryStats;
import com.swrobotics.blockauto.profiler.ProfileNode;
import com.swrobotics.blockauto.profiler.Profiler;

import imgui.ImGui;
import imgui.flag.ImGuiTableFlags;

import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL20;

public final class BlockAutoProfilerTool extends ProfilerTool {
    private final String vendor, renderer, version, glslVersion;

    public BlockAutoProfilerTool(BlockAuto log) {
        super(log, "BlockAuto Profiler");

        // Get OpenGL info for debugging
        vendor = GL11.glGetString(GL11.GL_VENDOR);
        renderer = GL11.glGetString(GL11.GL_RENDERER);
        version = GL11.glGetString(GL11.GL_VERSION);
        glslVersion = GL11.glGetString(GL20.GL_SHADING_LANGUAGE_VERSION);
    }

    @Override
    protected void showHeader() {
        ImGui.text("OpenGL info:");
        if (ImGui.beginTable("header", 2, ImGuiTableFlags.SizingStretchProp)) {
            ImGui.tableNextColumn();
            ImGui.text("Vendor:");
            ImGui.tableNextColumn();
            ImGui.text(vendor);

            ImGui.tableNextColumn();
            ImGui.text("Renderer:");
            ImGui.tableNextColumn();
            ImGui.text(renderer);

            ImGui.tableNextColumn();
            ImGui.text("Version:");
            ImGui.tableNextColumn();
            ImGui.text(version);

            ImGui.tableNextColumn();
            ImGui.text("GLSL Version:");
            ImGui.tableNextColumn();
            ImGui.text(glslVersion);

            ImGui.endTable();
        }

        ImGui.separator();
    }

    @Override
    protected ProfileNode getLastData() {
        return Profiler.getLastData();
    }

    @Override
    protected MemoryStats getMemStats() {
        return MemoryStats.current();
    }
}
