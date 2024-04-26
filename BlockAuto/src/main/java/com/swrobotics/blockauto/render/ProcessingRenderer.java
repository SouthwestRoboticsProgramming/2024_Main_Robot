package com.swrobotics.blockauto.render;

import processing.core.PConstants;
import processing.core.PGraphics;

import java.util.Stack;

public final class ProcessingRenderer implements Renderer2d {
    private final PGraphics graphics;
    private float strokeMul;
    private final Stack<Float> strokeMulStack;
    private boolean shapeIsPolygon;

    public ProcessingRenderer(PGraphics graphics) {
        this.graphics = graphics;
        strokeMul = 1;
        strokeMulStack = new Stack<>();
    }

    @Override
    public void pushTransform() {
        graphics.pushMatrix();
        strokeMulStack.push(strokeMul);
    }

    @Override
    public void popTransform() {
        graphics.popMatrix();
        strokeMul = strokeMulStack.pop();
    }

    @Override
    public void translate(double x, double y) {
        graphics.translate((float) x, (float) y);
    }

    @Override
    public void rotate(double angleRad) {
        graphics.rotate((float) angleRad);
    }

    @Override
    public void scale(double x, double y) {
        graphics.scale((float) x, (float) y);
        strokeMul /= Math.min(x, y);
    }

    @Override
    public void noStroke() {
        graphics.noStroke();
    }

    @Override
    public void setStroke(double r, double g, double b, double a) {
        graphics.stroke((float) r, (float) g, (float) b, (float) a);
    }

    @Override
    public void setStrokeWidth(double widthPx) {
        graphics.strokeWeight((float) widthPx * strokeMul);
    }

    @Override
    public void noFill() {
        graphics.noFill();
    }

    @Override
    public void setFill(double r, double g, double b, double a) {
        graphics.fill((float) r, (float) g, (float) b, (float) a);
    }

    @Override
    public void setShapeOrigin(ShapeOrigin origin) {
        int mode = origin == ShapeOrigin.TOP_LEFT ? PConstants.CORNER : PConstants.CENTER;
        graphics.rectMode(mode);
        graphics.ellipseMode(mode);
    }

    @Override
    public void line(double x1, double y1, double x2, double y2) {
        graphics.line((float) x1, (float) y1, (float) x2, (float) y2);
    }

    @Override
    public void rect(double x, double y, double w, double h) {
        graphics.rect((float) x, (float) y, (float) w, (float) h);
    }

    @Override
    public void ellipse(double x, double y, double w, double h) {
        graphics.ellipse((float) x, (float) y, (float) w, (float) h);
    }

    @Override
    public void beginShape(ShapeType type) {
        shapeIsPolygon = type == ShapeType.POLYGON;
        graphics.beginShape(shapeIsPolygon ? PConstants.POLYGON : PConstants.LINE_STRIP);
    }

    @Override
    public void vertex(double x, double y) {
        graphics.vertex((float) x, (float) y);
    }

    @Override
    public void endShape() {
        if (shapeIsPolygon) graphics.endShape(PConstants.CLOSE);
        else graphics.endShape();
    }
}
