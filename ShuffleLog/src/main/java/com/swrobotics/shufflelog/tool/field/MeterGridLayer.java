package com.swrobotics.shufflelog.tool.field;

import com.google.gson.JsonObject;
import com.swrobotics.shufflelog.json.JsonObj;
import imgui.ImGui;
import imgui.type.ImBoolean;

import processing.core.PGraphics;

public final class MeterGridLayer implements FieldLayer {
    private final ImBoolean show = new ImBoolean(true);

    @Override
    public String getName() {
        return "Grid";
    }

    @Override
    public void draw(PGraphics g) {
        if (!show.get()) return;

        float width = (float) FieldViewTool.WIDTH;
        float height = (float) FieldViewTool.HEIGHT;

        g.stroke(64);
        g.strokeWeight(1);
        for (float x = 0; x < width; x += 1) {
            g.line(x, 0, x, height);
        }
        for (float y = 0; y < height; y += 1) {
            g.line(0, y, width, y);
        }

        g.noFill();
        g.rect(0, 0, width, height);
    }

    @Override
    public void showGui() {
        ImGui.checkbox("Show", show);
    }

    @Override
    public void load(JsonObj obj) {
        show.set(obj.getBoolean("grid", true));
    }

    @Override
    public void store(JsonObject obj) {
        obj.addProperty("grid", show.get());
    }
}
