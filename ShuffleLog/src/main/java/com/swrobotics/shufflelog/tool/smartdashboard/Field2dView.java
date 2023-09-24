package com.swrobotics.shufflelog.tool.smartdashboard;

import com.swrobotics.shufflelog.render.Renderer2d;
import com.swrobotics.shufflelog.render.ShapeOrigin;
import com.swrobotics.shufflelog.render.ShapeType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class Field2dView {
    public final Map<String, Pose2d[]> poseSets;

    public Field2dView(NetworkTable table) {
        poseSets = new HashMap<>();

        for (String key : table.getKeys()) {
            if (key.startsWith(".")) continue;

            double[] components = table.getEntry(key).getDoubleArray(new double[0]);
            Pose2d[] poses = new Pose2d[components.length / 3];
            for (int i = 0; i < poses.length; i++) {
                poses[i] =
                        new Pose2d(
                                components[i * 3],
                                components[i * 3 + 1],
                                Rotation2d.fromDegrees(components[i * 3 + 2]));
            }

            poseSets.put(key, poses);
        }
    }

    private void stroke(Renderer2d r, float[] color) {
        r.setStroke(color[0] * 255, color[1] * 255, color[2] * 255);
    }

    public void render(Renderer2d renderer, Field2dSettings settings, boolean drawBorder) {
        if (drawBorder) {
            // Border
            renderer.noFill();
            renderer.setStroke(255);
            renderer.setStrokeWidth(2);
            renderer.rect(0, 0, settings.fieldWidth.get(), settings.fieldHeight.get());
        }

        List<String> sortedKeys = new ArrayList<>(poseSets.keySet());
        sortedKeys.sort(String.CASE_INSENSITIVE_ORDER);

        for (String name : sortedKeys) {
            Pose2d[] poses = poseSets.get(name);
            Field2dSettings.PoseSetSettings poseSettings = settings.poseSetSettings.get(name);

            // TODO: Track style

            // Lines
            stroke(renderer, poseSettings.lineColor);
            renderer.noFill();
            int style = poseSettings.style.get();
            if (style == Field2dSettings.PoseSetSettings.STYLE_LINE
                    || style == Field2dSettings.PoseSetSettings.STYLE_LINE_CLOSED) {
                renderer.beginShape(
                        style == Field2dSettings.PoseSetSettings.STYLE_LINE_CLOSED
                                ? ShapeType.POLYGON
                                : ShapeType.LINE_STRIP);
                for (Pose2d pose : poses) {
                    renderer.vertex(pose.getX(), pose.getY());
                }
                renderer.endShape();
            }

            // Boxes and arrows
            double width = poseSettings.boxWidth.get();
            double length = poseSettings.boxLength.get();
            double arrowSz = poseSettings.arrowSize.get();
            renderer.noFill();
            renderer.setShapeOrigin(ShapeOrigin.CENTER);
            for (Pose2d pose : poses) {
                renderer.pushTransform();
                renderer.translate(pose.getX(), pose.getY());
                renderer.rotate(pose.getRotation().getRadians());
                if (style == Field2dSettings.PoseSetSettings.STYLE_BOX) {
                    stroke(renderer, poseSettings.lineColor);
                    renderer.setStrokeWidth(poseSettings.lineWeight.get());
                    renderer.rect(0, 0, length, width);
                }
                if (poseSettings.arrows.get()) {
                    renderer.setStrokeWidth(poseSettings.arrowWeight.get());
                    stroke(renderer, poseSettings.arrowColor);
                    renderer.beginShape(ShapeType.POLYGON);
                    renderer.vertex(arrowSz * -length / 2, arrowSz * -width / 2);
                    renderer.vertex(arrowSz * -length / 2, arrowSz * width / 2);
                    renderer.vertex(arrowSz * length / 2, 0);
                    renderer.endShape();
                }
                renderer.popTransform();
            }
        }
    }
}
