package com.swrobotics.blockauto.tool;

import com.swrobotics.blockauto.BlockAuto;
import com.swrobotics.blockauto.Styles;
import com.swrobotics.blockauto.tool.smartdashboard.SmartDashboard;

import com.swrobotics.blockauto.tool.tetris.TetrisTool;
import imgui.ImGui;
import imgui.extension.implot.ImPlot;
import imgui.type.ImBoolean;

public final class MenuBarTool implements Tool {
    private final SmartDashboard smartDashboard;
    private final ImBoolean showDemo, showPlotDemo;
    private final ImBoolean plotDemoOpen;
    private final ImBoolean showTetris;

    private final TetrisTool tetris;

    public MenuBarTool(BlockAuto log, SmartDashboard smartDashboard) {
        this.smartDashboard = smartDashboard;

        showDemo = new ImBoolean(false);
        showPlotDemo = new ImBoolean(false);

        plotDemoOpen = new ImBoolean(true);
        showTetris = new ImBoolean(false);

        tetris = new TetrisTool(log);
    }

    @Override
    public void process() {
        if (ImGui.beginMainMenuBar()) {
            if (ImGui.beginMenu("Debug")) {
                ImGui.menuItem("Show demo", null, showDemo);
                ImGui.menuItem("Show plot demo", null, showPlotDemo);
                ImGui.menuItem("Show Tetris", null, showTetris);

                ImGui.endMenu();
            }

            if (ImGui.beginMenu("Styles")) {
                if (ImGui.menuItem("Switch to dark mode")) Styles.applyDarkColors();
                if (ImGui.menuItem("Switch to light mode")) Styles.applyLightColors();
                ImGui.endMenu();
            }

            smartDashboard.showMenu();

            ImGui.endMainMenuBar();
        }

        if (showDemo.get()) ImGui.showDemoWindow();
        if (showPlotDemo.get()) ImPlot.showDemoWindow(plotDemoOpen);
        if (showTetris.get()) tetris.process();
    }
}
