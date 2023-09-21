package com.swrobotics.shufflelog.render;

// RGBA colors are 0-255
public interface Renderer2d {
    void pushTransform();
    void popTransform();
    void translate(double x, double y);
    void rotate(double angleRad);
    default void scale(double s) { scale(s, s); }
    void scale(double x, double y);

    void noStroke();
    default void setStroke(double rgb) { setStroke(rgb, rgb, rgb, 255); }
    default void setStroke(double r, double g, double b) { setStroke(r, g, b, 255); }
    void setStroke(double r, double g, double b, double a);
    void setStrokeWidth(double widthPx);

    void noFill();
    default void setFill(double rgb) { setFill(rgb, rgb, rgb, 255); }
    default void setFill(double r, double g, double b) { setFill(r, g, b, 255); }
    void setFill(double r, double g, double b, double a);

    // Default origin is ShapeOrigin.TOP_LEFT
    void setShapeOrigin(ShapeOrigin origin);
    void line(double x1, double y1, double x2, double y2);
    void rect(double x, double y, double w, double h);
    void ellipse(double x, double y, double w, double h);

    void beginShape(ShapeType type);
    void vertex(double x, double y);
    void endShape();
}
