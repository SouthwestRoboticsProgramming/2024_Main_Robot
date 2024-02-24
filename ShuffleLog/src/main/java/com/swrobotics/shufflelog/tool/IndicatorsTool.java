package com.swrobotics.shufflelog.tool;

import com.swrobotics.shufflelog.tool.data.nt.NTInstanceListener;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import imgui.ImGui;

public final class ShooterReadyTool implements Tool, NTInstanceListener {
    private static final float[] READY = {0, 1, 0, 1};
    private static final float[] NOT_READY = {0.25f, 0, 0, 1};

    private BooleanSubscriber sub;

    @Override
    public void onNTInit(NetworkTableInstance inst) {
        sub = inst.getBooleanTopic("Shooter/Is Ready").subscribe(false);
    }

    @Override
    public void onNTClose() {
        sub = null;
    }

    @Override
    public void process() {
        if (ImGui.begin("Shooter Ready")) {
            boolean ready = sub != null && sub.get();

            float w = ImGui.getContentRegionAvailX();
            float h = ImGui.getContentRegionAvailY();

            ImGui.colorButton("##color", ready ? READY : NOT_READY, 0, w, h);
        }
        ImGui.end();
    }
}
