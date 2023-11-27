package com.swrobotics.shufflelog.tool.smartdashboard;

import java.util.Set;

public final class Field2dInfo {
    private final String name;
    private final Field2dSettings settings;
    private final Field2dView view;

    public Field2dInfo(String name, Field2dSettings settings, Field2dView view) {
        this.name = name;
        this.settings = settings;
        this.view = view;

        Set<String> published = view.poseSets.keySet();
        settings.update(published);
    }

    public String getName() {
        return name;
    }

    public Field2dSettings getSettings() {
        return settings;
    }

    public Field2dView getView() {
        return view;
    }
}
