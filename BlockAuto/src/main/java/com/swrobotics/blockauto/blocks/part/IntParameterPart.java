package com.swrobotics.blockauto.blocks.part;

import com.swrobotics.blockauto.util.ImGuiUtils;
import imgui.type.ImInt;

public final class IntParameterPart implements ParameterPart {
    private final String paramName;
    private final int defaultVal;

    public IntParameterPart(String paramName, int defaultVal) {
        this.paramName = paramName;
        this.defaultVal = defaultVal;
    }

    @Override
    public Object edit(Object value) {
        ImInt i = new ImInt((int) value);
        ImGuiUtils.autoSizingEditInt("##" + paramName, i);
        return i.get();
    }

    @Override
    public Object getDefaultValue() {
        return defaultVal;
    }
}
