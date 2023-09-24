package com.swrobotics.shufflelog.tool.field;

import com.google.gson.JsonObject;
import com.swrobotics.shufflelog.json.JsonObj;
import com.swrobotics.shufflelog.render.ProcessingRenderer;
import com.swrobotics.shufflelog.render.Renderer2d;
import com.swrobotics.shufflelog.tool.smartdashboard.Field2dInfo;
import com.swrobotics.shufflelog.tool.smartdashboard.Field2dSettings;
import com.swrobotics.shufflelog.tool.smartdashboard.Field2dView;
import com.swrobotics.shufflelog.tool.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import imgui.ImGui;
import imgui.flag.ImGuiDataType;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import processing.core.PGraphics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class Field2dLayer implements FieldLayer {
    private static final int UNIT_METERS = 0;
    private static final int UNIT_FEET = 1;
    private static final int UNIT_INCHES = 2;
    private static final String[] UNIT_NAMES = {"Meters", "Feet", "Inches"};

    private static final double FEET_TO_METERS = Units.feetToMeters(1);
    private static final double INCHES_TO_METERS = Units.inchesToMeters(1);

    private static final class FieldOverlay {
        final ImBoolean enabled = new ImBoolean(false);
        final ImInt unit = new ImInt(UNIT_METERS);
        final ImDouble offsetX = new ImDouble(0);
        final ImDouble offsetY = new ImDouble(0);

        boolean published;
        Field2dSettings settings;
        Field2dView view;

        boolean shouldRender() {
            return published && enabled.get();
        }
    }

    private final SmartDashboard smartDashboard;
    private final Map<String, FieldOverlay> overlays;
    private String highlightOverlay;

    public Field2dLayer(SmartDashboard smartDashboard) {
        this.smartDashboard = smartDashboard;
        overlays = new HashMap<>();
        highlightOverlay = null;
    }

    @Override
    public String getName() {
        return "Field2d";
    }

    @Override
    public void draw(PGraphics g) {
        for (Map.Entry<String, FieldOverlay> entry : overlays.entrySet()) {
            FieldOverlay overlay = entry.getValue();
            if (!overlay.shouldRender())
                continue;

            g.pushMatrix();
            Renderer2d renderer = new ProcessingRenderer(g);

            // Offsets are in meters always
            renderer.translate(overlay.offsetX.get(), overlay.offsetY.get());

            // Scale overlay rendering to correct units
            int unit = overlay.unit.get();
            if (unit == UNIT_FEET)
                renderer.scale(FEET_TO_METERS);
            else if (unit == UNIT_INCHES)
                renderer.scale(INCHES_TO_METERS);

            // Only show border if highlighted
            overlay.view.render(renderer, overlay.settings, entry.getKey().equals(highlightOverlay));
            g.popMatrix();

            // Spacing between layers
            g.translate(0, 0, FieldViewTool.LAYER_Z_SPACING);
        }
    }

    @Override
    public void processAlways() {
        // Set all published false
        for (FieldOverlay overlay : overlays.values()) {
            overlay.published = false;
        }

        for (Field2dInfo info : smartDashboard.getAllField2d()) {
            FieldOverlay overlay = overlays.computeIfAbsent(info.getName(), (n) -> new FieldOverlay());

            // Only set published back to true if found in SmartDashboard
            overlay.published = true;
            overlay.settings = info.getSettings();
            overlay.view = info.getView();
        }
    }

    @Override
    public void showGui() {
        // TODO: Mouse picking for offset location

        List<String> keys = new ArrayList<>(overlays.keySet());
        keys.sort(String.CASE_INSENSITIVE_ORDER);
        highlightOverlay = null;
        for (String name : keys) {
            FieldOverlay overlay = overlays.get(name);

            ImGui.beginDisabled(!overlay.published);
            if (!overlay.published)
                ImGui.setNextItemOpen(false);

            boolean open = ImGui.treeNodeEx(name);
            if (ImGui.isItemHovered())
                highlightOverlay = name;
            if (open) {
                ImGui.checkbox("Show", overlay.enabled);
                ImGui.combo("Unit", overlay.unit, UNIT_NAMES);
                ImGui.inputScalar("Offset X", ImGuiDataType.Double, overlay.offsetX);
                ImGui.inputScalar("Offset Y", ImGuiDataType.Double, overlay.offsetY);
                ImGui.spacing();
                if (overlay.settings != null && overlay.view != null)
                    overlay.settings.edit(overlay.view.poseSets.keySet());

                ImGui.treePop();
            }
            ImGui.endDisabled();
        }
    }

    @Override
    public int getSubLayerCount() {
        int count = 0;
        for (FieldOverlay overlay : overlays.values()) {
            if (overlay.shouldRender())
                count++;
        }
        return count;
    }

    @Override
    public void load(JsonObj obj) {
        JsonObj field2d = obj.getObject("field2d");
        for (String key : field2d.keySet()) {
            JsonObj overlayObj = field2d.getObject(key);

            FieldOverlay overlay = new FieldOverlay();
            overlay.enabled.set(overlayObj.getBoolean("enabled", false));
            overlay.unit.set(overlayObj.getInt("unit", UNIT_METERS));
            overlay.offsetX.set(overlayObj.getDouble("offsetX", 0));
            overlay.offsetY.set(overlayObj.getDouble("offsetY", 0));

            overlays.put(key, overlay);
        }
    }

    @Override
    public void store(JsonObject obj) {
        JsonObject field2d = new JsonObject();
        for (Map.Entry<String, FieldOverlay> entry : overlays.entrySet()) {
            FieldOverlay overlay = entry.getValue();

            JsonObject overlayObj = new JsonObject();
            overlayObj.addProperty("enabled", overlay.enabled.get());
            overlayObj.addProperty("unit", overlay.unit.get());
            overlayObj.addProperty("offsetX", overlay.offsetX.get());
            overlayObj.addProperty("offsetY", overlay.offsetY.get());

            field2d.add(entry.getKey(), overlayObj);
        }
        obj.add("field2d", field2d);
    }
}
