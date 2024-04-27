package com.swrobotics.blockauto.blocks.part;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.type.ImInt;

import java.util.List;

public final class EnumParameterPart implements ParameterPart {
    private final String paramName;
    private final String[] enumValues;
    private final int defaultValueIdx;

    public EnumParameterPart(String paramName, List<String> enumValues, String defaultValue) {
        this.paramName = paramName;
        this.enumValues = enumValues.toArray(new String[0]);

        int defaultValueIdx = -1;
        int idx = 0;
        for (String value : enumValues) {
            if (value.equals(defaultValue)) {
                defaultValueIdx = idx;
                break;
            }
            idx++;
        }

        if (defaultValueIdx == -1) {
            System.err.println("Invalid default enum value in parameter " + paramName);
            defaultValueIdx = 0;
        }

        this.defaultValueIdx = defaultValueIdx;
    }

    @Override
    public Object edit(Object value) {
        ImInt i = new ImInt((int) value);

        ImVec2 textSize = new ImVec2();
        float maxWidth = 0;
        for (String v : enumValues) {
            ImGui.calcTextSize(textSize, v);
            maxWidth = Math.max(textSize.x, maxWidth);
        }

        ImGui.setNextItemWidth(maxWidth + 2 * ImGui.getStyle().getFramePaddingX() + ImGui.getFrameHeight());
        ImGui.combo("##" + paramName, i, enumValues);
        return i.get();
    }

    @Override
    public Object getDefaultValue() {
        return defaultValueIdx;
    }
}
