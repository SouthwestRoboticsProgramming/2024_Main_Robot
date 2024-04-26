package com.swrobotics.blockauto.tool;

import com.swrobotics.blockauto.tool.data.nt.NTInstanceListener;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import imgui.ImGui;
import imgui.flag.ImGuiColorEditFlags;

public final class IndicatorsTool implements Tool, NTInstanceListener {
    private static final float[] NOTE = {1, 0.5f, 0, 1};
    private static final float[] NO_NOTE = {0.25f, 0.125f, 0, 1};

    private static final float[] READY = {0, 1, 0, 1};
    private static final float[] NOT_READY = {0.25f, 0, 0, 1};

    private NetworkTableEntry hasPieceEntry;
    private NetworkTableEntry shooterReadyEntry;

    @Override
    public void onNTInit(NetworkTableInstance inst) {
        // For some reason getting the entry directly doesn't work
        hasPieceEntry = inst.getTable("Indexer").getEntry("Has Piece");
        shooterReadyEntry = inst.getTable("Shooter").getEntry("Is Ready");
    }

    @Override
    public void onNTClose() {
        hasPieceEntry = null;
        shooterReadyEntry = null;
    }

    @Override
    public void process() {
        if (ImGui.begin("Indicators")) {
            if (shooterReadyEntry == null) {
                ImGui.textDisabled("Disconnected");
                ImGui.end();
                return;
            }

            boolean hasPiece = hasPieceEntry.getBoolean(false);
            boolean shooterReady = shooterReadyEntry.getBoolean(false);

            float w = ImGui.getContentRegionAvailX() / 2 - 5;
            float h = ImGui.getContentRegionAvailY();

            int flags = ImGuiColorEditFlags.NoTooltip;
            ImGui.colorButton("##note", hasPiece ? NOTE : NO_NOTE, flags, w, h);
            ImGui.sameLine();
            ImGui.colorButton("##shooter", shooterReady ? READY : NOT_READY, flags, w, h);
        }
        ImGui.end();
    }
}
