package com.swrobotics.shufflelog;

import com.google.gson.GsonBuilder;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.swrobotics.messenger.client.MessengerClient;
import com.swrobotics.shufflelog.json.JsonObj;
import com.swrobotics.shufflelog.profiler.Profiler;
import com.swrobotics.shufflelog.tool.MenuBarTool;
import com.swrobotics.shufflelog.tool.PreMatchChecklistTool;
import com.swrobotics.shufflelog.tool.IndicatorsTool;
import com.swrobotics.shufflelog.tool.Tool;
import com.swrobotics.shufflelog.tool.data.DataLogTool;
import com.swrobotics.shufflelog.tool.data.nt.NetworkTablesTool;
import com.swrobotics.shufflelog.tool.field.FieldViewTool;
import com.swrobotics.shufflelog.tool.messenger.MessengerTool;
import com.swrobotics.shufflelog.tool.profile.ShuffleLogProfilerTool;
import com.swrobotics.shufflelog.tool.sftp.SftpTool;
import com.swrobotics.shufflelog.tool.smartdashboard.SmartDashboard;
import com.swrobotics.shufflelog.tool.taskmanager.RoboRIOFilesTool;
import com.swrobotics.shufflelog.tool.taskmanager.TaskManagerTool;
import com.swrobotics.shufflelog.util.ExpressionInput;

import edu.wpi.first.math.WPIMathJNI;
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

public final class ShuffleLog extends PApplet {
    public static boolean SIM_MODE;

    private static final String LAYOUT_FILE = "layout.ini";
    private static final String PERSISTENCE_FILE = "persistence.json";
    private static final int THREAD_POOL_SIZE = 4;

    private final ImGuiImplGlfw imGuiGlfw = new ImGuiImplGlfw();
    private final ImGuiImplGl3 imGuiGl3 = new ImGuiImplGl3();
    private ImPlotContext imPlotCtx;

    private final List<Tool> tools = new ArrayList<>();
    private final List<Tool> addedTools = new ArrayList<>();
    private final List<Tool> removedTools = new ArrayList<>();

    // Things shared between tools
    private JsonObj persistence;
    //    private Properties persistence;
    private final ExecutorService threadPool = Executors.newFixedThreadPool(THREAD_POOL_SIZE);
    private MessengerClient messenger;

    private long startTime;

    @Override
    public void settings() {
        NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
        WPIMathJNI.Helper.setExtractOnStaticLoad(false);
        try {
            CombinedRuntimeLoader.loadLibraries(
                    ShuffleLog.class, "wpiutiljni", "wpimathjni", "ntcorejni");
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

    // FIXME: Make this one actually work
    private void saveDefaultLayout() {
        File defaultLayoutFile = new File(LAYOUT_FILE);
        if (!defaultLayoutFile.exists()) {
            try {
                InputStream in = getClass().getClassLoader().getResourceAsStream(LAYOUT_FILE);
                OutputStream out = new FileOutputStream(defaultLayoutFile);

                if (in == null) throw new IOException("Failed to load default layout resource");

                byte[] buf = new byte[1024];
                int read;
                while ((read = in.read(buf)) > 0) out.write(buf, 0, read);

                in.close();
                out.close();
            } catch (IOException e) {
                System.err.println("Failed to save default layout file:");
                e.printStackTrace();
            }
        }
    }

    @Override
    public void setup() {
        saveDefaultLayout();

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

        // Set default font
        try {
            textFont(
                    new PFont(
                            getClass()
                                    .getClassLoader()
                                    .getResourceAsStream("fonts/PTSans-Regular-14.vlw")));
        } catch (IOException e) {
            e.printStackTrace();
        }

        SmartDashboard smartDashboard = new SmartDashboard();
        NetworkTablesTool nt = new NetworkTablesTool(threadPool);
        nt.addListener(smartDashboard);

        tools.add(new MenuBarTool(smartDashboard));
        MessengerTool msg = new MessengerTool(this);
        tools.add(msg);
        tools.add(new ShuffleLogProfilerTool(this));

        DataLogTool dataLog = new DataLogTool(this);
        tools.add(dataLog);
        tools.add(nt);
        tools.add(smartDashboard);

        tools.add(new TaskManagerTool(this, "TaskManager"));
        tools.add(new RoboRIOFilesTool(this));
        tools.add(new FieldViewTool(this, smartDashboard, nt));
        if (!SIM_MODE) tools.add(new PreMatchChecklistTool(msg));

        tools.add(new SftpTool(threadPool));

        IndicatorsTool ready = new IndicatorsTool();
        nt.addListener(ready);
        tools.add(ready);

        for (Tool tool : tools) {
            tool.load(persistence);
        }

        startTime = System.currentTimeMillis();
    }

    @Override
    public void draw() {
        Profiler.beginMeasurements("Root");

        if (messenger != null) {
            Profiler.push("Read Messages");
            messenger.readMessages();
            Profiler.pop();
        }

        Profiler.push("Begin GUI frame");
        imGuiGlfw.flushEvents();
        imGuiGlfw.newFrame();
        ImGui.newFrame();
        ImGuizmo.beginFrame();
        ExpressionInput.newFrame();
        Profiler.pop();

        background(210);
        ImGui.dockSpaceOverViewport();

        for (Tool tool : tools) {
            Profiler.push(tool.getClass().getSimpleName());

            // if (going to crash) { dont(); }
            try {
                tool.process();
            } catch (Throwable t) {
                // Log it and ignore
                t.printStackTrace();
            }

            Profiler.pop();
        }
        tools.addAll(addedTools);
        tools.removeAll(removedTools);
        addedTools.clear();
        removedTools.clear();

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
        messenger.disconnect();

        ImPlot.destroyContext(imPlotCtx);
        ImGui.destroyContext();

        JsonObject persistence = new JsonObject();
        for (Tool tool : tools) {
            tool.store(persistence);
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

    public void addTool(Tool tool) {
        addedTools.add(tool);
    }

    public void removeTool(Tool tool) {
        removedTools.add(tool);
    }

    public double getTimestamp() {
        return (System.currentTimeMillis() - startTime) / 1000.0;
    }

    public MessengerClient getMessenger() {
        return messenger;
    }

    public void setMessenger(MessengerClient messenger) {
        this.messenger = messenger;
    }

    public static void main(String[] args) {
        SIM_MODE = args.length >= 1 && args[0].equals("sim");
        PApplet.main(ShuffleLog.class);
    }
}
