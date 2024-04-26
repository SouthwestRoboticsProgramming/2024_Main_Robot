package com.swrobotics.blockauto.tool.field;

import com.google.gson.JsonObject;
import com.swrobotics.blockauto.json.JsonObj;
import edu.wpi.first.math.util.Units;
import imgui.ImGui;
import imgui.type.ImBoolean;
import processing.core.PConstants;
import processing.core.PGraphics;

public final class FieldVectorLayer2024 implements FieldLayer {
    // Defined dimensions
    private static final float fieldWidth = (float) FieldViewTool.WIDTH;
    private static final float fieldHeight = (float) FieldViewTool.HEIGHT;

    private static final float topToSpeaker = 1.614f;
    private static final float allianceToSpeakerFront = 0.919f;
    private static final float speakerMajorBase = 2.099f;
    private static final float speakerMinorBase = 1.0287f;

    private static final float sourceInsetFromBottom = 1.063f;
    private static final float sourceInsetFromAlliance = 1.808163f;

    private static final float stageFootPerpendicular = 0.3556f;
    private static final float stageFootParallel = 0.31115f;
    private static final float stageLegHorizontalToAllianceWall = 3.21945f;
    private static final float stageHorizontalSpan = 2.595906f + 0.012153f + 0.006350f;

    private static final float noteInnerDiameter = (float) Units.inchesToMeters(10);
    private static final float noteOuterDiameter = (float) Units.inchesToMeters(14);
    private static final float allianceNoteFromWall = (float) Units.inchesToMeters(114);
    private static final float allianceNoteSpacing = (float) Units.inchesToMeters(57);
    private static final float centerNoteSpacing = (float) Units.inchesToMeters(66);

    private static final float wingTapeX = 5.872505f;
    private static final float sourceTapeOffset = 0.476727f;
    private static final float ampTapeFromTop = 0.450850f;
    private static final float ampTapeFromAlliance = 3.302000f;

    private static final float ampOpeningFromAlliance = 1.538097f;
    private static final float ampOpeningWidth = 0.609600f;
    private static final float ampDepth = 0.098425f;

    // Derived dimensions
    private static final float speakerBaseHalfDiff = (speakerMajorBase - speakerMinorBase) / 2;
    private static final float stageTruncatedCornerHeight = (float) Math.sqrt(3) / 2 * stageFootPerpendicular;
    private static final float stageCornerToCenterDist = (stageTruncatedCornerHeight + stageHorizontalSpan) * 2 / 3;
    private static final float stageTruncatedCornerToCenterDist = stageCornerToCenterDist - stageTruncatedCornerHeight;
    private static final float stageCenterX = stageLegHorizontalToAllianceWall + stageTruncatedCornerToCenterDist;
    private static final float noteDiameter = (noteInnerDiameter + noteOuterDiameter) / 2;

    private final ImBoolean show;

    public FieldVectorLayer2024() {
        show = new ImBoolean();
    }

    @Override
    public String getName() {
        return "Field Vector (2024)";
    }

    private void drawFieldHalf(PGraphics g, boolean isRedAlliance) {
        if (isRedAlliance) {
            g.pushMatrix();
            g.translate(fieldWidth / 2, 0);
            g.scale(-1, 1);
            g.translate(-fieldWidth / 2, 0);
        }

        int red = g.color(255, 0, 0);
        int blue = g.color(0, 0, 255);
        int allianceColor = isRedAlliance ? red : blue;
        int oppositeColor = isRedAlliance ? blue : red;

        // Field perimeter
        g.strokeWeight(2);
        g.stroke(255);
        g.noFill();
        g.beginShape(PConstants.LINE_STRIP);
        g.vertex(fieldWidth/2, fieldHeight);
        g.vertex(0, fieldHeight);
        g.vertex(0, sourceInsetFromBottom);
        g.vertex(sourceInsetFromAlliance, 0);
        g.vertex(fieldWidth/2, 0);
        g.endShape();

        // Tape
        g.strokeWeight(2);
        g.stroke(allianceColor);
        g.line(wingTapeX, 0, wingTapeX, fieldHeight);
        g.beginShape(PConstants.LINE_STRIP);
        g.vertex(0, fieldHeight - ampTapeFromTop);
        g.vertex(ampTapeFromAlliance, fieldHeight - ampTapeFromTop);
        g.vertex(ampTapeFromAlliance, fieldHeight);
        g.endShape();
        g.stroke(128);
        g.line(sourceInsetFromAlliance, fieldHeight - ampTapeFromTop, sourceInsetFromAlliance, 0);
        g.stroke(oppositeColor);
        g.beginShape(PConstants.LINE_STRIP);
        g.vertex(0, sourceInsetFromBottom + sourceTapeOffset);
        g.vertex(sourceInsetFromAlliance, sourceTapeOffset);
        g.vertex(sourceInsetFromAlliance, 0);
        g.endShape();

        // Speaker trapezoid
        g.strokeWeight(2);
        g.stroke(allianceColor);
        g.noFill();
        g.beginShape(PConstants.LINE_STRIP);
        g.vertex(0, fieldHeight - topToSpeaker);
        g.vertex(allianceToSpeakerFront, fieldHeight - topToSpeaker - speakerBaseHalfDiff);
        g.vertex(allianceToSpeakerFront, fieldHeight - topToSpeaker - speakerBaseHalfDiff - speakerMinorBase);
        g.vertex(0, fieldHeight - topToSpeaker - speakerMajorBase);
        g.endShape();

        // Stage border

        float[] stageOuterPtRotated = rotateStage(-stageTruncatedCornerToCenterDist + stageFootParallel/2, 0, 1);
        if (!isRedAlliance) {
//            System.out.println("Stage foot center pos: " + stageOuterPtRotated[0] + ", " + stageOuterPtRotated[1]);
        }
        // 0.74235576, 1.2857977
        // 4.8597364

        g.pushMatrix();
        g.translate(stageCenterX, fieldHeight/2);
        g.beginShape(PConstants.LINE_LOOP);
        for (int i = 0; i < 3; i++) {
            stageVertex(g, -stageTruncatedCornerToCenterDist, -stageFootPerpendicular/2, i);
            stageVertex(g, -stageTruncatedCornerToCenterDist, stageFootPerpendicular/2, i);
        }
        g.endShape();

        // Stage feet
        g.strokeWeight(1);
        g.stroke(255);
        for (int i = 0; i < 3; i++) {
            g.pushMatrix();
            g.rotate(stageAngle(i));
            g.rect(-stageTruncatedCornerToCenterDist, -stageFootPerpendicular/2, stageFootParallel, stageFootPerpendicular);
            g.popMatrix();
        }
        g.popMatrix();

        // Amp
        g.strokeWeight(2);
        g.stroke(allianceColor);
        g.rect(ampOpeningFromAlliance, fieldHeight, ampOpeningWidth, ampDepth);

        // Alliance note starting positions
        g.strokeWeight(3);
        g.stroke(255, 128, 0);
        g.ellipseMode(PConstants.CENTER);
        for (int i = 0; i < 3; i++) {
            g.ellipse(allianceNoteFromWall, fieldHeight / 2 + i * allianceNoteSpacing, noteDiameter, noteDiameter);
        }

        if (isRedAlliance)
            g.popMatrix();
    }

    private void stageVertex(PGraphics g, float relX, float relY, int i) {
        float[] pos = rotateStage(relX, relY, i);
        g.vertex(pos[0], pos[1]);
    }

    private float stageAngle(int i) {
        return (float) -Math.PI * 2 / 3 * i;
    }

    private float[] rotateStage(float relX, float relY, int i) {
        float a = stageAngle(i);
        float sin = (float) Math.sin(a);
        float cos = (float) Math.cos(a);

        return new float[] {
                cos * relX - sin * relY,
                sin * relX + cos * relY
        };
    }

    @Override
    public void draw(PGraphics g) {
        if (!show.get())
            return;

        drawFieldHalf(g, false);
        drawFieldHalf(g, true);

        // Center line
        g.stroke(255);
        g.strokeWeight(2);
        g.line(fieldWidth/2, 0, fieldWidth/2, fieldHeight);

        // Center note starting positions
        g.strokeWeight(3);
        g.stroke(255, 128, 0);
        g.ellipseMode(PConstants.CENTER);
        for (int i = 0; i < 5; i++) {
            g.ellipse(fieldWidth/2, fieldHeight / 2 - (i - 2) * centerNoteSpacing, noteDiameter, noteDiameter);
        }
    }

    @Override
    public void showGui() {
        ImGui.checkbox("Show", show);
    }

    @Override
    public void load(JsonObj obj) {
        show.set(obj.getBoolean("vector", true));
    }

    @Override
    public void store(JsonObject obj) {
        obj.addProperty("vector", show.get());
    }
}
