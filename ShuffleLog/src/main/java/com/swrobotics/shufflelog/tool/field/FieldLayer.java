package com.swrobotics.shufflelog.tool.field;

import com.google.gson.JsonObject;
import com.swrobotics.shufflelog.json.JsonObj;
import processing.core.PGraphics;

public interface FieldLayer {
    /**
     * Gets the name of this layer for use in ImGui. This name is required to be unique per layer.
     *
     * @return name
     */
    String getName();

    /** Called every frame regardless of whether the GUI is visible. */
    default void processAlways() {}

    /**
     * Draws the content of this layer. The expected units are in meters.
     *
     * @param g graphics to draw with
     */
    void draw(PGraphics g);

    /** Draws the ImGui content for this layer. */
    void showGui();

    default int getSubLayerCount() { return 1; }
}
