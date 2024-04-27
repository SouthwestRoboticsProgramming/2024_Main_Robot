package com.swrobotics.blockauto.blocks.part;

import com.swrobotics.blockauto.util.ImGuiUtils;
import imgui.type.ImString;

public final class StringParameterPart implements ParameterPart {
    private final String paramName;
    private final String defaultVal;

    public StringParameterPart(String paramName, String defaultVal) {
        this.paramName = paramName;
        this.defaultVal = defaultVal;
    }

    @Override
    public Object edit(Object value) {
        ImString str = new ImString(128);
        str.set((String) value);
        ImGuiUtils.autoSizingEditString("##" + paramName, str);
        return str.get();
    }

    @Override
    public Object getDefaultValue() {
        return defaultVal;
    }
}
