package com.swrobotics.shufflelog.tool.smartdashboard;

import com.swrobotics.shufflelog.render.Renderer2d;
import edu.wpi.first.networktables.NetworkTable;
import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.List;

public final class Mechanism2dView {
    public final double width, height;
    public final Vector3f backgroundColor;
    public final Root[] roots;

    public Mechanism2dView(NetworkTable table) {
        double[] dims = table.getEntry("dims").getDoubleArray(new double[] {1, 1});
        width = dims.length >= 1 ? dims[0] : 1;
        height = dims.length >= 2 ? dims[1] : 1;
        backgroundColor = colorFromHex(table.getEntry("backgroundColor").getString("#000000"));

        List<String> subTables = new ArrayList<>(table.getSubTables());
        subTables.sort(String.CASE_INSENSITIVE_ORDER); // Sort by alphabetical for consistency with Glass
        int i = 0;
        roots = new Root[subTables.size()];
        for (String subTable : subTables) {
            roots[i++] = new Root(table.getSubTable(subTable));
        }
    }

    private static Ligament[] getLigaments(NetworkTable table) {
        List<String> subTables = new ArrayList<>(table.getSubTables());
        subTables.sort(String.CASE_INSENSITIVE_ORDER); // Sort by alphabetical for consistency with Glass

        int i = 0;
        Ligament[] ligaments = new Ligament[subTables.size()];
        for (String subTable : subTables) {
            ligaments[i++] = new Ligament(table.getSubTable(subTable));
        }
        return ligaments;
    }

    public static final class Root {
        public final double x, y;

        public final Ligament[] ligaments;

        public Root(NetworkTable table) {
            x = table.getEntry("x").getDouble(0);
            y = table.getEntry("y").getDouble(0);
            ligaments = getLigaments(table);
        }
    }

    private static Vector3f colorFromHex(String hex) {
        try {
            return new Vector3f(
                    Integer.parseInt(hex, 1, 3, 16),
                    Integer.parseInt(hex, 3, 5, 16),
                    Integer.parseInt(hex, 5, 7, 16));
        } catch (NumberFormatException e) {
            return new Vector3f(255, 0, 255);
        }
    }

    public static final class Ligament {
        public final double angle;
        public final Vector3f color;
        public final double length;
        public final double weight;

        public final Ligament[] children;

        public Ligament(NetworkTable table) {
            angle = table.getEntry("angle").getDouble(0);
            color = colorFromHex(table.getEntry("color").getString("#ffffff"));
            length = table.getEntry("length").getDouble(1);
            weight = table.getEntry("weight").getDouble(1);
            children = getLigaments(table);
        }
    }

    private void renderLigament(Renderer2d r, Ligament l) {
        r.pushTransform();
        r.rotate(Math.toRadians(l.angle));
        r.setStroke(l.color.x, l.color.y, l.color.z);
        r.setStrokeWidth(l.weight);
        r.line(0, 0, l.length, 0);
        r.translate(l.length, 0);
        for (Ligament child : l.children) {
            renderLigament(r, child);
        }
        r.popTransform();
    }

    // Assumes coordinate system is typical (+X right, +Y up)
    public void render(Renderer2d r) {
        r.noStroke();
        r.setFill(backgroundColor.x, backgroundColor.y, backgroundColor.z);
        r.rect(0, 0, width, height);

        for (Root root : roots) {
            r.pushTransform();
            r.translate(root.x, root.y);
            for (Ligament l : root.ligaments) {
                renderLigament(r, l);
            }
            r.popTransform();
        }
    }
}
