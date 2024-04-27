package com.swrobotics.blockauto;

import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.swrobotics.blockauto.json.JsonObj;
import com.swrobotics.blockauto.profiler.Profiler;
import com.swrobotics.blockauto.view.MenuBarView;
import com.swrobotics.blockauto.view.View;
import com.swrobotics.blockauto.view.blocks.BlockCanvasView;
import com.swrobotics.blockauto.view.blocks.BlockPaletteView;
import com.swrobotics.blockauto.view.nt.NTInstanceListener;
import com.swrobotics.blockauto.view.nt.NetworkTablesView;
import com.swrobotics.blockauto.view.field.FieldView;
import com.swrobotics.blockauto.util.ExpressionInput;

import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;

import imgui.ImGui;
import imgui.ImGuiIO;
import imgui.extension.imguizmo.ImGuizmo;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.ImPlotContext;
import imgui.flag.ImGuiConfigFlags;
import imgui.gl3.ImGuiImplGl3;

import processing.core.PApplet;
import processing.core.PFont;

import java.io.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public final class BlockAuto extends PApplet {
    private static final String LAYOUT_FILE = "layout.ini";
    private static final String PERSISTENCE_FILE = "persistence.json";
    private static final int THREAD_POOL_SIZE = 4;

    private final ImGuiImplGlfw imGuiGlfw = new ImGuiImplGlfw();
    private final ImGuiImplGl3 imGuiGl3 = new ImGuiImplGl3();
    private ImPlotContext imPlotCtx;

    private final List<View> views = new ArrayList<>();
    private final List<View> addedViews = new ArrayList<>();
    private final List<View> removedViews = new ArrayList<>();

    // Things shared between tools
    private JsonObj persistence;
    private final ExecutorService threadPool = Executors.newFixedThreadPool(THREAD_POOL_SIZE);

    private long startTime;

    @Override
    public void settings() {
        NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
        WPIMathJNI.Helper.setExtractOnStaticLoad(false);
        try {
            CombinedRuntimeLoader.loadLibraries(
                    BlockAuto.class, "wpiutiljni", "wpimathjni", "ntcorejni");
        } catch (IOException e) {
            throw new RuntimeException("Failed to load WPILib libraries", e);
        }

        try (FileReader reader = new FileReader(PERSISTENCE_FILE)) {
            persistence = new JsonObj(JsonParser.parseReader(reader).getAsJsonObject());
        } catch (IOException e) {
            System.err.println("Failed to load persistence file, using defaults");
            persistence = new JsonObj(null);
        }

        JsonObj window = persistence.getObject("window");
        int width = window.getInt("width", 1280);
        int height = window.getInt("height", 720);
        size(width, height, P2D);
    }

    private void initTools() {
        NetworkTablesView nt = new NetworkTablesView(threadPool);

        views.add(new MenuBarView());
        views.add(nt);
        views.add(new FieldView(this));
        views.add(new BlockPaletteView());
        views.add(new BlockCanvasView());
    }

    @Override
    public void setup() {
        surface.setResizable(true);
        long windowHandle = (long) surface.getNative();

        ImGui.createContext();
        imPlotCtx = ImPlot.createContext();

        ImGuiIO io = ImGui.getIO();
        io.setIniFilename(LAYOUT_FILE);
        io.setConfigFlags(ImGuiConfigFlags.DockingEnable);
        Styles.applyStyling();
        Styles.applyDarkColors();

        imGuiGlfw.init(windowHandle, true);
        imGuiGl3.init();

        // Set default Processing font
        try {
            textFont(
                    new PFont(
                            getClass()
                                    .getClassLoader()
                                    .getResourceAsStream("fonts/PTSans-Regular-14.vlw")));
        } catch (IOException e) {
            e.printStackTrace();
        }

        initTools();
        for (View view : views) {
            view.load(persistence);
        }

        startTime = System.currentTimeMillis();
    }

    @Override
    public void draw() {
        Profiler.beginMeasurements("Root");

        Profiler.push("Begin GUI frame");
        imGuiGlfw.flushEvents();
        imGuiGlfw.newFrame();
        ImGui.newFrame();
        ImGuizmo.beginFrame();
        ExpressionInput.newFrame();
        Profiler.pop();

        background(210);
//        ImGui.dockSpaceOverViewport();

        for (View view : views) {
            Profiler.push(view.getClass().getSimpleName());
            view.process();
            Profiler.pop();
        }
        views.addAll(addedViews);
        views.removeAll(removedViews);
        addedViews.clear();
        removedViews.clear();

        Profiler.push("Render GUI");
        Profiler.push("Flush");
        flush();
        Profiler.pop();
        Profiler.push("Render draw data");
        ImGui.render();
        imGuiGl3.renderDrawData(ImGui.getDrawData());
        Profiler.pop();
        Profiler.pop();

        Profiler.endMeasurements();
    }

    @Override
    public void exit() {
        ImPlot.destroyContext(imPlotCtx);
        ImGui.destroyContext();

        JsonObject persistence = new JsonObject();
        for (View view : views) {
            view.store(persistence);
        }

        try (FileWriter writer = new FileWriter(PERSISTENCE_FILE)) {
            new GsonBuilder().setPrettyPrinting().create().toJson(persistence, writer);
        } catch (IOException e) {
            System.err.println("Failed to save persistent data");
            e.printStackTrace();
        }

        super.exit();
    }

    @Override
    public void keyPressed() {
        // Prevent closing on escape key press
        if (key == ESC) key = 0;
    }

    public void addTool(View view) {
        addedViews.add(view);
    }

    public void removeTool(View view) {
        removedViews.add(view);
    }

    public double getTimestamp() {
        return (System.currentTimeMillis() - startTime) / 1000.0;
    }

    public static void main(String[] args) {
        PApplet.main(BlockAuto.class);
    }
}
