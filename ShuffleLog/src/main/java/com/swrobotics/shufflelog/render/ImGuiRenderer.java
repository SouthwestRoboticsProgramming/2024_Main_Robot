package com.swrobotics.shufflelog.render;

import imgui.ImDrawList;
import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImDrawFlags;

import org.joml.Matrix3x2fStack;
import org.joml.Vector2f;

import java.util.ArrayList;
import java.util.List;

public final class ImGuiRenderer implements Renderer2d {
    private final ImDrawList draw;

    private boolean stroke, fill;
    private int strokeColor, fillColor;
    private float strokeWidth;
    private ShapeOrigin shapeOrigin;

    private ShapeType shapeType;
    private final List<ImVec2> shapeVertices;

    private final Matrix3x2fStack transformStack;

    public ImGuiRenderer(ImDrawList draw) {
        this.draw = draw;
        stroke = false;
        fill = false;
        strokeWidth = 1;
        transformStack = new Matrix3x2fStack(32);
        shapeType = null;
        shapeVertices = new ArrayList<>();
    }

    @Override
    public void pushTransform() {
        transformStack.pushMatrix();
    }

    @Override
    public void popTransform() {
        transformStack.popMatrix();
    }

    @Override
    public void translate(double x, double y) {
        transformStack.translate((float) x, (float) y);
    }

    @Override
    public void rotate(double angleRad) {
        transformStack.rotate((float) angleRad);
    }

    @Override
    public void scale(double x, double y) {
        transformStack.scale((float) x, (float) y);
    }

    @Override
    public void noStroke() {
        stroke = false;
    }

    private int color(double r, double g, double b, double a) {
        return ImGui.getColorU32(
                (float) r / 255, (float) g / 255, (float) b / 255, (float) a / 255);
    }

    @Override
    public void setStroke(double r, double g, double b, double a) {
        stroke = true;
        strokeColor = color(r, g, b, a);
    }

    @Override
    public void setStrokeWidth(double widthPx) {
        strokeWidth = (float) widthPx;
    }

    @Override
    public void noFill() {
        fill = false;
    }

    @Override
    public void setFill(double r, double g, double b, double a) {
        fill = true;
        fillColor = color(r, g, b, a);
    }

    @Override
    public void setShapeOrigin(ShapeOrigin origin) {
        shapeOrigin = origin;
    }

    private Vector2f transform(double x, double y) {
        return transformStack.transformPosition(new Vector2f((float) x, (float) y));
    }

    @Override
    public void line(double x1, double y1, double x2, double y2) {
        if (!stroke) return;

        Vector2f start = transform(x1, y1);
        Vector2f end = transform(x2, y2);
        System.out.println("Line from " + start + " to " + end);
        draw.addLine(start.x, start.y, end.x, end.y, strokeColor, strokeWidth);
    }

    @Override
    public void rect(double x, double y, double w, double h) {
//        throw new UnsupportedOperationException("TODO: Implement this");
    }

    @Override
    public void ellipse(double x, double y, double w, double h) {
        throw new UnsupportedOperationException("TODO: Implement this");
    }

    @Override
    public void beginShape(ShapeType type) {
        if (shapeType != null) throw new IllegalStateException("Shape already began");
        shapeType = type;
    }

    @Override
    public void vertex(double x, double y) {
        if (shapeType == null) throw new IllegalStateException("Shape not started");
        Vector2f pos = transform(x, y);
        shapeVertices.add(new ImVec2(pos.x, pos.y));
    }

    @Override
    public void endShape() {
        if (shapeType == null) throw new IllegalStateException("Shape not started");

        ImVec2[] pos = shapeVertices.toArray(new ImVec2[0]);
        shapeVertices.clear();

        if (shapeType == ShapeType.POLYGON && fill)
            draw.addConvexPolyFilled(pos, pos.length, fillColor);
        if (stroke)
            draw.addPolyline(
                    pos,
                    pos.length,
                    strokeColor,
                    shapeType == ShapeType.POLYGON ? ImDrawFlags.Closed : 0,
                    strokeWidth);
    }
}
