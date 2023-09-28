package com.swrobotics.shufflelog.json;

import com.google.gson.JsonObject;

import java.util.Collections;
import java.util.Set;

public final class JsonObj {
    private final JsonObject obj;

    public JsonObj(JsonObject obj) {
        this.obj = obj;
    }

    public JsonObj getObject(String key) {
        return new JsonObj(obj == null ? null : obj.getAsJsonObject(key));
    }

    public JsonArr getArray(String key) {
        return new JsonArr(obj == null ? null : obj.getAsJsonArray(key));
    }

    public boolean getBoolean(String key, boolean def) {
        if (obj == null || !obj.has(key)) return def;
        return obj.get(key).getAsBoolean();
    }

    public int getInt(String key, int def) {
        if (obj == null || !obj.has(key)) return def;
        return obj.get(key).getAsInt();
    }

    public double getDouble(String key, double def) {
        if (obj == null || !obj.has(key)) return def;
        return obj.get(key).getAsDouble();
    }

    public Set<String> keySet() {
        return obj == null ? Collections.emptySet() : obj.keySet();
    }

    public JsonObject getGsonObj() {
        return obj;
    }
}
