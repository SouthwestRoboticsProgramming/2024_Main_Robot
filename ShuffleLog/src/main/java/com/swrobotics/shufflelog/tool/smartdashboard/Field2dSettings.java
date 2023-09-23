package com.swrobotics.shufflelog.tool.smartdashboard;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.swrobotics.shufflelog.json.JsonArr;
import com.swrobotics.shufflelog.json.JsonObj;
import imgui.ImGui;
import imgui.flag.ImGuiDataType;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;

import java.util.*;

public final class Field2dSettings {
    public static final int UNITS_METERS = 0;
    public static final int UNITS_FEET = 1;
    public static final int UNITS_INCHES = 2;
    public static final String[] UNITS_NAMES = {"Meters", "Feet", "Inches"};

    public final ImInt units;
    public final ImDouble fieldWidth;
    public final ImDouble fieldHeight;

    public final Map<String, PoseSetSettings> poseSetSettings;

    public Field2dSettings(JsonObj obj) {
        poseSetSettings = new HashMap<>();

        units = new ImInt(obj.getInt("units", UNITS_METERS));
        fieldWidth = new ImDouble(obj.getDouble("width", 16));
        fieldHeight = new ImDouble(obj.getDouble("height", 8));

        JsonObj poseSets = obj.getObject("poses");
        for (String name : poseSets.keySet()) {
            PoseSetSettings settings = new PoseSetSettings(poseSets.getObject(name));
            poseSetSettings.put(name, settings);
        }
    }

    private static JsonArray colorToArray(float[] color) {
        JsonArray array = new JsonArray(3);
        array.add(color[0]);
        array.add(color[1]);
        array.add(color[2]);
        return array;
    }

    public void update(Set<String> publishedPoseSets) {
        for (String set : publishedPoseSets) {
            if (!poseSetSettings.containsKey(set)) {
                poseSetSettings.put(set, new PoseSetSettings(new JsonObj(null)));
            }
        }
    }

    public void edit(Set<String> poseSetsToShow) {
        ImGui.combo("Units", units, UNITS_NAMES);
        ImGui.inputScalar("Field width", ImGuiDataType.Double, fieldWidth);
        ImGui.inputScalar("Field height", ImGuiDataType.Double, fieldHeight);

        List<String> keys = new ArrayList<>(poseSetsToShow);
        keys.sort(String.CASE_INSENSITIVE_ORDER);

        for (String key : keys) {
            if (ImGui.treeNodeEx(key)) {
                poseSetSettings.get(key).edit();
                ImGui.treePop();
            }
        }
    }

    public JsonObject save() {
        JsonObject obj = new JsonObject();
        obj.addProperty("units", units.get());
        obj.addProperty("width", fieldWidth.get());
        obj.addProperty("height", fieldHeight.get());

        JsonObject poseSets = new JsonObject();
        for (Map.Entry<String, PoseSetSettings> entry : poseSetSettings.entrySet()) {
            poseSets.add(entry.getKey(), entry.getValue().save());
        }
        obj.add("poses", poseSets);

        return obj;
    }

    public static final class PoseSetSettings {
        public static final int STYLE_BOX = 0;
        public static final int STYLE_LINE = 1;
        public static final int STYLE_LINE_CLOSED = 2;
        public static final int STYLE_TRACK = 3;
        public static final int STYLE_HIDDEN = 4;
        public static final String[] STYLE_NAMES = {"Box", "Line", "Line (Closed)", "Track", "Hidden"};

        public final ImInt style;
        public final ImDouble boxWidth;
        public final ImDouble boxLength;
        public final ImDouble lineWeight;
        public final float[] lineColor;
        public final ImBoolean arrows;
        public final ImDouble arrowSize;
        public final ImDouble arrowWeight;
        public final float[] arrowColor;

        private float[] getColor(JsonArr arr, float r, float g, float b) {
            float[] color = {r, g, b};
            int i = 0;
            for (JsonElement elem : arr) {
                color[i++] = elem.getAsFloat();
                if (i >= 3)
                    break;
            }
            return color;
        }

        public PoseSetSettings(JsonObj obj) {
            style = new ImInt(obj.getInt("style", STYLE_BOX));
            boxWidth = new ImDouble(obj.getDouble("boxWidth", 0.686));
            boxLength = new ImDouble(obj.getDouble("boxLength", 0.82));
            lineWeight = new ImDouble(obj.getDouble("lineWeight", 2));
            lineColor = getColor(obj.getArray("lineColor"), 1, 0, 0);
            arrows = new ImBoolean(obj.getBoolean("arrows", true));
            arrowSize = new ImDouble(obj.getDouble("arrowSize", 0.5));
            arrowWeight = new ImDouble(obj.getDouble("arrowWeight", 2));
            arrowColor = getColor(obj.getArray("arrowColor"), 0, 1, 0);
        }

        public void edit() {
            ImGui.combo("Style", style, STYLE_NAMES);
            int style = this.style.get();
            if (style == STYLE_BOX) {
                ImGui.inputScalar("Box width", ImGuiDataType.Double, boxWidth);
                ImGui.inputScalar("Box length", ImGuiDataType.Double, boxLength);
            }
            if (style != STYLE_HIDDEN) {
                ImGui.inputScalar("Line weight", ImGuiDataType.Double, lineWeight);
                ImGui.colorEdit3("Line color", lineColor);
            }
            ImGui.spacing();
            ImGui.checkbox("Arrows", arrows);
            if (arrows.get()) {
                ImGui.sliderScalar("Arrow size", ImGuiDataType.Double, arrowSize, 0, 1);
                ImGui.inputScalar("Arrow weight", ImGuiDataType.Double, arrowWeight);
                ImGui.colorEdit3("Arrow color", arrowColor);
            }
        }

        public JsonObject save() {
            JsonObject obj = new JsonObject();
            obj.addProperty("style", style.get());
            obj.addProperty("boxWidth", boxWidth.get());
            obj.addProperty("boxLength", boxLength.get());
            obj.addProperty("lineWeight", lineWeight.get());
            obj.add("lineColor", colorToArray(lineColor));
            obj.addProperty("arrows", arrows.get());
            obj.addProperty("arrowSize", arrowSize.get());
            obj.addProperty("arrowWeight", 2);
            obj.add("arrowColor", colorToArray(arrowColor));
            return obj;
        }
    }
}
