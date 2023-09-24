package com.swrobotics.shufflelog.tool.smartdashboard;

public final class Field2dInfo {
    private final Field2dSettings settings;
    private final Field2dView view;

    public Field2dInfo(Field2dSettings settings, Field2dView view) {
        this.settings = settings;
        this.view = view;
    }

    public Field2dSettings getSettings() {
        return settings;
    }

    public Field2dView getView() {
        return view;
    }
}
