package com.swrobotics.blockauto.tool;

import com.google.gson.JsonObject;
import com.swrobotics.blockauto.json.JsonObj;

public interface Tool {
    void process();

    default void load(JsonObj obj) {}

    default void store(JsonObject obj) {}
}
