package com.swrobotics.blockauto.util;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import imgui.type.ImString;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public final class ImGuiUtils {
    public static float calcFrameWidth(String text) {
        ImVec2 textSize = new ImVec2();
        ImGui.calcTextSize(textSize, text);
        return textSize.x + 2 * ImGui.getStyle().getFramePaddingX();
    }

    public static boolean autoSizingEditString(String label, ImString str) {
        ImGui.setNextItemWidth(calcFrameWidth(str.get()));
        return ImGui.inputText(label, str, ImGuiInputTextFlags.NoHorizontalScroll);
    }

    private static final Map<Integer, ImString> inputBuffers = new HashMap<>();
    private static ImString previewBuffer = new ImString(128);

    public static boolean autoSizingEditDouble(String label, ImDouble val) {
        return autoSizingEdit(label, String.format("%.6f", val.get()), (str) -> {
            try {
                double result = Double.parseDouble(str);
                val.set(result);
            } catch (NumberFormatException e) {
                // Ignore and not set
            }
        });
    }

    public static boolean autoSizingEditInt(String label, ImInt val) {
        return autoSizingEdit(label, String.format("%d", val.get()), (str) -> {
            try {
                int result = Integer.parseInt(str);
                val.set(result);
            } catch (NumberFormatException e) {
                // Ignore and not set
            }
        });
    }

    private static boolean autoSizingEdit(String label, String formattedCurrent, Consumer<String> resultHandler) {
        int id = ImGui.getID(label);
        ImString existingState = inputBuffers.get(id);

        ImString inputBuffer;
        if (existingState != null) {
            inputBuffer = existingState;
        } else {
            inputBuffer = previewBuffer;
            inputBuffer.set(formattedCurrent);
        }

        boolean changed = autoSizingEditString(label, inputBuffer);
        boolean active = ImGui.isItemActive();

        if (active || changed) {
            ImString state;
            if (existingState != null) {
                state = existingState;
            } else {
                state = previewBuffer;
                previewBuffer = new ImString(128);
                inputBuffers.put(id, state);
            }

            resultHandler.accept(state.get());
        }

        if (!active)
            inputBuffers.remove(id);

        return changed;
    }
}
