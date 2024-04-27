package com.swrobotics.blockauto.view;

import com.google.gson.JsonObject;
import com.swrobotics.blockauto.json.JsonObj;

public interface View {
    void process();

    default void load(JsonObj obj) {}

    default void store(JsonObject obj) {}
}
