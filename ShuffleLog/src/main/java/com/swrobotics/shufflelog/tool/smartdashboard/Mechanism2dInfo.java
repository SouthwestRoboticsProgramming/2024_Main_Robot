package com.swrobotics.shufflelog.tool.smartdashboard;

public final class Mechanism2dInfo {
    private final String name;
    private final Mechanism2dView view;

    public Mechanism2dInfo(String name, Mechanism2dView view) {
        this.name = name;
        this.view = view;
    }

    public String getName() {
        return name;
    }

    public Mechanism2dView getView() {
        return view;
    }
}
