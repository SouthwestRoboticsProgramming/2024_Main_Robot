package com.swrobotics.blockauto.tool.field;

import com.google.gson.JsonObject;
import com.swrobotics.blockauto.json.JsonObj;
import com.swrobotics.blockauto.render.ProcessingRenderer;
import com.swrobotics.blockauto.render.Renderer2d;
import com.swrobotics.blockauto.tool.smartdashboard.*;
import com.swrobotics.blockauto.util.ExpressionInput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

import imgui.ImGui;
import imgui.ImGuiViewport;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;

import imgui.type.ImString;
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

    private static final int PLANE_XY = 0;
    private static final int PLANE_XZ = 1;
    private static final int PLANE_YZ = 2;
    private static final String[] PLANE_NAMES = {"XY", "XZ", "YZ"};

    private static final double FEET_TO_METERS = Units.feetToMeters(1);
    private static final double INCHES_TO_METERS = Units.inchesToMeters(1);

    private static final class FieldOverlay {
        final ImBoolean enabled = new ImBoolean(false);
        final ImInt unit = new ImInt(UNIT_METERS);
        final ImDouble offsetX = new ImDouble(0);
        final ImDouble offsetY = new ImDouble(0);

        final Map<String, MechanismOverlay> mechanisms = new HashMap<>();

        boolean published;
        Field2dSettings settings;
        Field2dView view;

        boolean shouldRender() {
            return published && enabled.get();
        }
    }

    private static final class MechanismOverlay {
        final ImBoolean enabled = new ImBoolean(false);

        final ImString parentPoseSet = new ImString(64);
        final ImInt parentPoseIdx = new ImInt(0);

        final ImDouble offsetX = new ImDouble(0);
        final ImDouble offsetY = new ImDouble(0);
        final ImDouble offsetZ = new ImDouble(0);
        final ImInt plane = new ImInt(PLANE_XZ);

        boolean published;
        Mechanism2dView view;

        boolean shouldRender() {
            return published && enabled.get();
        }
    }

    private final SmartDashboard smartDashboard;
    private final Map<String, FieldOverlay> overlays;
    private String highlightOverlay;

    private final ImString newMechName = new ImString(64);
    private final ImString newMechPoseSet = new ImString(64);
    private final ImInt newMechPoseIdx = new ImInt(0);

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
            if (!overlay.shouldRender()) continue;

            g.pushMatrix();
            Renderer2d renderer = new ProcessingRenderer(g);

            // Offsets are in meters always
            renderer.translate(overlay.offsetX.get(), overlay.offsetY.get());

            // Scale overlay rendering to correct units
            int unit = overlay.unit.get();
            if (unit == UNIT_FEET) renderer.scale(FEET_TO_METERS);
            else if (unit == UNIT_INCHES) renderer.scale(INCHES_TO_METERS);

            // Only show border if highlighted
            overlay.view.render(
                    renderer, overlay.settings, entry.getKey().equals(highlightOverlay));

            for (MechanismOverlay mech : overlay.mechanisms.values()) {
                if (!mech.shouldRender())
                    continue;

                Pose2d[] poses = overlay.view.poseSets.get(mech.parentPoseSet.get());
                int poseIdx = mech.parentPoseIdx.get();
                if (poses == null || poseIdx < 0 || poseIdx >= poses.length)
                    continue;

                Pose2d parentPose = poses[poseIdx];
                g.pushMatrix();
                g.translate((float) parentPose.getX(), (float) parentPose.getY());
                g.rotate((float) parentPose.getRotation().getRadians());

                g.translate(
                        (float) mech.offsetX.get(),
                        (float) mech.offsetY.get(),
                        (float) mech.offsetZ.get()
                );

                switch (mech.plane.get()) {
                    case PLANE_YZ:
                        g.rotateY((float) Math.PI / 2);
                        // Fall through
                    case PLANE_XZ:
                        g.rotateX((float) Math.PI / 2);
                        break;
                }

                mech.view.render(renderer, false);
                g.popMatrix();
            }

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

            for (MechanismOverlay mechanism : overlay.mechanisms.values()) {
                mechanism.published = false;
            }
        }

        List<Mechanism2dInfo> mechanism2dInfos = smartDashboard.getAllMechanism2d();
        for (Field2dInfo info : smartDashboard.getAllField2d()) {
            FieldOverlay overlay =
                    overlays.computeIfAbsent(info.getName(), (n) -> new FieldOverlay());

            // Only set published back to true if found in SmartDashboard
            overlay.published = true;
            overlay.settings = info.getSettings();
            overlay.view = info.getView();

            for (Mechanism2dInfo mechInfo : mechanism2dInfos) {
                System.out.println(mechInfo.getName());
                System.out.println(overlay.mechanisms);
                MechanismOverlay mechOverlay = overlay.mechanisms.get(mechInfo.getName());
                if (mechOverlay == null)
                    continue;

                mechOverlay.published = true;
                mechOverlay.view = mechInfo.getView();
            }
        }
    }

    @Override
    public void showGui() {
        List<String> keys = new ArrayList<>(overlays.keySet());
        keys.sort(String.CASE_INSENSITIVE_ORDER);
        highlightOverlay = null;
        for (String name : keys) {
            FieldOverlay overlay = overlays.get(name);

            ImGui.beginDisabled(!overlay.published);
            if (!overlay.published) ImGui.setNextItemOpen(false);

            boolean open = ImGui.treeNodeEx(name);
            if (ImGui.isItemHovered()) highlightOverlay = name;
            if (open) {
                ImGui.checkbox("Show", overlay.enabled);
                ImGui.combo("Unit", overlay.unit, UNIT_NAMES);
                ExpressionInput.inputDouble("Offset X", overlay.offsetX);
                ExpressionInput.inputDouble("Offset Y", overlay.offsetY);
                ImGui.spacing();
                if (overlay.settings != null && overlay.view != null)
                    overlay.settings.edit(overlay.view.poseSets.keySet());

                ImGui.spacing();
                if (ImGui.treeNodeEx("Mechanisms")) {
                    List<String> mechKeys = new ArrayList<>(overlay.mechanisms.keySet());
                    mechKeys.sort(String.CASE_INSENSITIVE_ORDER);
                    String mechToRemove = null;
                    for (String mechName : mechKeys) {
                        MechanismOverlay mech = overlay.mechanisms.get(mechName);

                        if (ImGui.treeNodeEx(mechName)) {
                            ImGui.checkbox("Show", mech.enabled);
                            ExpressionInput.inputDouble("Offset X", mech.offsetX);
                            ExpressionInput.inputDouble("Offset Y", mech.offsetY);
                            ExpressionInput.inputDouble("Offset Z", mech.offsetZ);
                            ImGui.combo("Plane", mech.plane, PLANE_NAMES);
                            ImGui.inputText("Parent Pose Set", mech.parentPoseSet);
                            ExpressionInput.inputInt("Parent Pose Idx", mech.parentPoseIdx);

                            if (ImGui.button("Remove")) {
                                mechToRemove = mechName;
                            }

                            ImGui.treePop();
                        }
                    }
                    if (mechToRemove != null) {
                        overlay.mechanisms.remove(mechToRemove);
                    }

                    ImGui.spacing();
                    if (ImGui.button("Add...")) {
                        newMechName.set("");
                        newMechPoseSet.set("Robot");
                        newMechPoseIdx.set(0);
                        ImGui.openPopup("Add Mechanism");
                    }

                    if (ImGui.beginPopupModal("Add Mechanism", ImGuiWindowFlags.NoMove)) {
                        // Center the popup
                        ImGuiViewport vp = ImGui.getWindowViewport();
                        ImGui.setWindowPos(
                                vp.getCenterX() - ImGui.getWindowWidth() / 2,
                                vp.getCenterY() - ImGui.getWindowHeight() / 2);

                        ImGui.text("Mechanism name:");
                        ImGui.setNextItemWidth(300);
                        ImGui.inputText("##name", newMechName);
                        ImGui.setItemDefaultFocus();

                        ImGui.text("Pose set:");
                        ImGui.setNextItemWidth(300);
                        ImGui.inputText("##pose_set", newMechPoseSet);

                        ImGui.text("Pose idx:");
                        ImGui.setNextItemWidth(300);
                        ExpressionInput.inputInt("##pose_idx", newMechPoseIdx);

                        if (ImGui.button("Add")) {
                            MechanismOverlay mech = new MechanismOverlay();
                            mech.parentPoseSet.set(newMechPoseSet.get());
                            mech.parentPoseIdx.set(newMechPoseIdx.get());

                            overlay.mechanisms.put(newMechName.get(), mech);
                            ImGui.closeCurrentPopup();
                        }
                        ImGui.sameLine();
                        if (ImGui.button("Cancel")) {
                            ImGui.closeCurrentPopup();
                        }

                        ImGui.endPopup();
                    }

                    ImGui.treePop();
                }

                ImGui.treePop();
            }
            ImGui.endDisabled();
        }
    }

    @Override
    public int getSubLayerCount() {
        int count = 0;
        for (FieldOverlay overlay : overlays.values()) {
            if (overlay.shouldRender()) count++;
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

            JsonObj mechanisms = overlayObj.getObject("mechanisms");
            for (String mechKey : mechanisms.keySet()) {
                JsonObj mechObj = mechanisms.getObject(mechKey);

                MechanismOverlay mechOverlay = new MechanismOverlay();
                mechOverlay.enabled.set(mechObj.getBoolean("enabled", true));
                mechOverlay.offsetX.set(mechObj.getDouble("offsetX", 0));
                mechOverlay.offsetY.set(mechObj.getDouble("offsetY", 0));
                mechOverlay.offsetZ.set(mechObj.getDouble("offsetZ", 0));
                mechOverlay.parentPoseSet.set(mechObj.getString("parentPoseSet", "Robot"));
                mechOverlay.parentPoseIdx.set(mechObj.getInt("parentPoseIdx", 0));
                mechOverlay.plane.set(mechObj.getInt("plane", PLANE_XZ));

                overlay.mechanisms.put(mechKey, mechOverlay);
            }

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

            JsonObject mechanisms = new JsonObject();
            for (Map.Entry<String, MechanismOverlay> mechEntry : overlay.mechanisms.entrySet()) {
                MechanismOverlay mech = mechEntry.getValue();

                JsonObject mechObj = new JsonObject();
                mechObj.addProperty("enabled", mech.enabled.get());
                mechObj.addProperty("offsetX", mech.offsetX.get());
                mechObj.addProperty("offsetY", mech.offsetY.get());
                mechObj.addProperty("offsetZ", mech.offsetZ.get());
                mechObj.addProperty("parentPoseSet", mech.parentPoseSet.get());
                mechObj.addProperty("parentPoseIdx", mech.parentPoseIdx.get());
                mechObj.addProperty("plane", mech.plane.get());

                mechanisms.add(mechEntry.getKey(), mechObj);
            }
            overlayObj.add("mechanisms", mechanisms);

            field2d.add(entry.getKey(), overlayObj);
        }
        obj.add("field2d", field2d);
    }
}
