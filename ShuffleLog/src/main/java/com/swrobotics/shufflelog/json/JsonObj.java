package com.swrobotics.shufflelog.json;

import com.google.gson.JsonObject;

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

    public int getInt(String key, int def) {
        if (obj == null || !obj.has(key)) return def;
        return obj.get(key).getAsInt();
    }

    public JsonObject getGsonObj() {
        return obj;
    }
}
