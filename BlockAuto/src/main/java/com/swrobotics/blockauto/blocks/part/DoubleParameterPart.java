package com.swrobotics.blockauto.blocks.part;

import com.swrobotics.blockauto.util.ImGuiUtils;
import imgui.type.ImDouble;

public final class DoubleParameterPart implements ParameterPart {
    private final String paramName;
    private final double defaultVal;

    public DoubleParameterPart(String paramName, double defaultVal) {
        this.paramName = paramName;
        this.defaultVal = defaultVal;
    }

    @Override
    public Object edit(Object value) {
        ImDouble d = new ImDouble((double) value);
        ImGuiUtils.autoSizingEditDouble("##" + paramName, d);
        return d.get();
    }

    @Override
    public Object getDefaultValue() {
        return defaultVal;
    }
}
