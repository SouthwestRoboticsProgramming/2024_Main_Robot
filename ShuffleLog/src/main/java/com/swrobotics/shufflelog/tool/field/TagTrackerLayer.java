package com.swrobotics.shufflelog.tool.field;

import com.swrobotics.shufflelog.tool.data.nt.NTInstanceListener;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import processing.core.PConstants;
import processing.core.PGraphics;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

public final class TagTrackerLayer implements FieldLayer, NTInstanceListener {
    private static final String TABLE_NAME = "/TagTracker";

    private NetworkTable table;

    @Override
    public void onNTInit(NetworkTableInstance inst) {
        table = inst.getTable(TABLE_NAME);
    }

    @Override
    public void onNTClose() {
        table = null;
    }

    @Override
    public String getName() {
        return "TagTracker";
    }

    private void drawAxes(PGraphics g) {
        g.stroke(255, 0, 0);
        g.line(0, 0, 0, 1, 0, 0);
        g.stroke(0, 255, 0);
        g.line(0, 0, 0, 0, 1, 0);
        g.stroke(0, 0, 255);
        g.line(0, 0, 0, 0, 0, 1);
    }

    private Pose3d unpackPose(int i, double[] data) {
        return new Pose3d(new Translation3d(data[i], data[i + 1], data[i + 2]),
                new Rotation3d(new Quaternion(data[i + 3], data[i + 4], data[i + 5], data[i + 6])));
    }

    @Override
    public void draw(PGraphics g) {
        if (table == null)
            return;

        double[] envData = table.getEntry("environment").getDoubleArray(new double[0]);

        Set<Integer> visibleIds = new HashSet<>();
        NetworkTable cameras = table.getSubTable("Cameras");
        for (String subTable : cameras.getSubTables()) {
            NetworkTable t = cameras.getSubTable(subTable);

            double[] poseData = t.getEntry("poses").getDoubleArray(new double[1]); // Have one entry (containing 0) by default

            int poseCount = (int) poseData[0];
            Pose3d bestPose = null;
            double bestError = Double.POSITIVE_INFINITY;
            for (int i = 0; i < poseCount; i++) {
                double error = poseData[i * 8 + 1];
                Pose3d pose = unpackPose(i * 8 + 2, poseData);
                if (error < bestError) {
                    bestError = error;
                    bestPose = pose;
                }
            }

            if (bestPose != null) {
                Translation3d tx = bestPose.getTranslation();
                Vector<N3> axis = bestPose.getRotation().getAxis();

                g.pushMatrix();
                g.translate((float) tx.getX(), (float) tx.getY(), (float) tx.getZ());
//                float angle = (float) ((System.currentTimeMillis() % 1000) / 1000.0f * 2 * Math.PI);
                g.rotate((float) bestPose.getRotation().getAngle(), (float) axis.get(0, 0), (float) axis.get(1, 0), (float) axis.get(2, 0));
                drawAxes(g);
//                axis = axis.div(axis.norm());
//                g.line(0, 0, 0, (float) axis.get(0, 0), (float) axis.get(1, 0), (float) axis.get(2, 0));
                g.popMatrix();
            }

//            int[] visibleIds = new int[poseData.length - poseCount * 8 - 1];
            int idCount = poseData.length - poseCount * 8 - 1;
            for (int i = 0; i < idCount; i++) {
//                visibleIds[i] = (int) poseData[poseData.length - visibleIds.length + i];
                visibleIds.add((int) poseData[poseData.length - idCount + i]);
            }
        }

        for (int i = 0; i < envData.length; i += 8) {
            int id = (int) envData[i];
            boolean visible = visibleIds.contains(id);

            Pose3d pose = unpackPose(i + 1, envData);
            Translation3d tx = pose.getTranslation();
            Vector<N3> axis = pose.getRotation().getAxis();

            g.pushMatrix();
            g.translate((float) tx.getX(), (float) tx.getY(), (float) tx.getZ());
            g.rotate((float) pose.getRotation().getAngle(), (float) axis.get(0, 0), (float) axis.get(1, 0), (float) axis.get(2, 0));
            g.fill(visible ? 128 : 64);
            g.stroke(255);
            g.strokeWeight(2);
            g.beginShape();
            g.vertex(0, -0.25f, -0.25f);
            g.vertex(0, -0.25f, 0.25f);
            g.vertex(0, 0.25f, 0.25f);
            g.vertex(0, 0.25f, -0.25f);
            g.endShape(PConstants.CLOSE);
            drawAxes(g);
            g.popMatrix();
        }
    }

    @Override
    public void showGui() {

    }
}
