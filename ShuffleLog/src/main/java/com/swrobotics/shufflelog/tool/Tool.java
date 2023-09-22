package com.swrobotics.shufflelog.tool;

import com.google.gson.JsonObject;
import com.swrobotics.shufflelog.json.JsonObj;

public interface Tool {
    void process();

    default void load(JsonObj obj) {}
    default void store(JsonObject obj) {}
}
