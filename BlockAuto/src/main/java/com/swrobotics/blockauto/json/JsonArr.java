package com.swrobotics.blockauto.json;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;

import java.util.Collections;
import java.util.Iterator;

public final class JsonArr implements Iterable<JsonElement> {
    private final JsonArray array;

    public JsonArr(JsonArray array) {
        this.array = array;
    }

    @Override
    public Iterator<JsonElement> iterator() {
        return array == null ? Collections.emptyIterator() : array.iterator();
    }
}
