package com.swrobotics.blockauto.view.field;

import com.google.gson.JsonObject;
import com.swrobotics.blockauto.json.JsonObj;

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
    void draw(PGraphics g, float strokeMul);

    /** Draws the ImGui content for this layer. */
    void showGui();

    default void load(JsonObj obj) {}

    default void store(JsonObject obj) {}
}
