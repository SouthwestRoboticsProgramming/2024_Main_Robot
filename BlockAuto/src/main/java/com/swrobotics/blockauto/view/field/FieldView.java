package com.swrobotics.blockauto.view.field;

import static com.swrobotics.blockauto.util.ProcessingUtils.setPMatrix;

import static processing.core.PConstants.P3D;

import com.google.gson.JsonObject;
import com.swrobotics.blockauto.BlockAuto;
import com.swrobotics.blockauto.json.JsonObj;
import com.swrobotics.blockauto.view.ViewportView;
import com.swrobotics.blockauto.util.SmoothFloat;

import imgui.ImGui;
import imgui.ImGuiIO;
import imgui.ImVec2;
import imgui.flag.ImGuiMouseButton;
import imgui.flag.ImGuiTableFlags;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImInt;

import org.joml.Matrix4f;
import org.joml.Vector2f;
import org.joml.Vector3f;
import org.joml.Vector4f;

import processing.core.PConstants;
import processing.core.PGraphics;
import processing.opengl.PGraphicsOpenGL;

import java.util.ArrayList;
import java.util.List;

public final class FieldView extends ViewportView {
    public static final double WIDTH = 16.541;
    public static final double HEIGHT = 8.211;

    private final List<FieldLayer> layers;

    private float prevMouseX, prevMouseY;

    public FieldView(BlockAuto log) {
        super(
                log,
                "Field View",
                ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoScrollWithMouse);

        layers = new ArrayList<>();
        layers.add(new MeterGridLayer());
        layers.add(new FieldVectorLayer2024());
    }

    private float calcReqScale(float px, float field) {
        return (px - 10) / field;
    }

    @Override
    protected void drawViewportContent(PGraphics g) {
        float normalScale =
                Math.min(
                        calcReqScale(g.width, (float) WIDTH),
                        calcReqScale(g.height, (float) HEIGHT));
        float rotatedScale =
                Math.min(
                        calcReqScale(g.width, (float) HEIGHT),
                        calcReqScale(g.height, (float) WIDTH));

        float scale;
        if (normalScale > rotatedScale) {
            scale = normalScale;
        } else {
            scale = rotatedScale;
        }

        g.translate(g.width / 2f, g.height / 2f);
        g.scale(scale, -scale);
        g.translate((float) -WIDTH/2, (float) -HEIGHT/2);

        g.background(0);
        g.noLights();

        float strokeMul = 1 / scale;

        for (FieldLayer layer : layers) {
            g.rectMode(PConstants.CORNER);
            g.ellipseMode(PConstants.CORNER);
            g.pushMatrix();
            layer.draw(g, strokeMul);
            g.popMatrix();
        }
    }

    @Override
    protected void drawGuiContent() {
        ImVec2 size = ImGui.getContentRegionAvail();
        drawViewport(size.x, size.y, false);

         ImGui.tableNextColumn();
            ImGui.beginChild("scroller");
            for (FieldLayer layer : layers) {
                if (ImGui.collapsingHeader(layer.getName())) {
                    ImGui.pushID(layer.getName());
                    ImGui.indent();
                    layer.showGui();
                    ImGui.unindent();
                    ImGui.popID();
                }
            }
            ImGui.endChild();
    }

    @Override
    public void process() {
        for (FieldLayer layer : layers)
            layer.processAlways();

        super.process();
    }

    @Override
    public void load(JsonObj obj) {
        JsonObj fieldViewObj = obj.getObject("field");
        for (FieldLayer layer : layers) {
            layer.load(fieldViewObj);
        }
    }

    @Override
    public void store(JsonObject obj) {
        JsonObject fieldViewObj = new JsonObject();
        for (FieldLayer layer : layers) {
            layer.store(fieldViewObj);
        }
        obj.add("field", fieldViewObj);
    }
}
