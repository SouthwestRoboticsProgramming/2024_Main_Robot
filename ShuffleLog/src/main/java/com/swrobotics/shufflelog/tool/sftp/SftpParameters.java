package com.swrobotics.shufflelog.tool.sftp;

import com.google.gson.JsonObject;
import com.swrobotics.shufflelog.json.JsonObj;
import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import imgui.type.ImString;

public final class SftpParameters {
    private final ImString name;

    private final ImString host;
    private final ImInt port;

    private final ImString username;
    private final ImBoolean hasPassword;
    private final ImString password;

    private final ImBoolean showPassword;

    public SftpParameters() {
        name = new ImString(64);
        host = new ImString(128);
        port = new ImInt();
        username = new ImString(64);
        hasPassword = new ImBoolean();
        password = new ImString(128);
        showPassword = new ImBoolean(false);
    }

    public void load(JsonObj json) {
        name.set(json.getString("name", "SFTP"));
        host.set(json.getString("host", "10.21.29.2"));
        port.set(json.getInt("port", 22));
        username.set(json.getString("username", "admin"));
        hasPassword.set(json.getBoolean("hasPassword", false));
        password.set(json.getString("password", ""));
    }

    public JsonObject save() {
        JsonObject obj = new JsonObject();
        obj.addProperty("name", name.get());
        obj.addProperty("host", host.get());
        obj.addProperty("port", port.get());
        obj.addProperty("username", username.get());
        obj.addProperty("hasPassword", hasPassword.get());
        if (hasPassword.get())
            obj.addProperty("password", password.get());
        return obj;
    }

    public String getName() {
        return name.get();
    }

    public String getHost() {
        return host.get();
    }

    public int getPort() {
        return port.get();
    }

    public String getUsername() {
        return username.get();
    }

    // Nullable
    public String getPassword() {
        return hasPassword.get() ? password.get() : null;
    }

    private void label(String label) {
        ImGui.tableNextColumn();
        ImGui.text(label);
        ImGui.tableNextColumn();
    }

    public void edit() {
        if (ImGui.beginTable("params", 2)) {
            if (ImGui.isWindowAppearing()) {
                showPassword.set(false);
            }

            label("Name:");
            ImGui.inputText("##name", name);
            ImGui.spacing();
            label("Host:");
            ImGui.inputText("##host", host);
            label("Port:");
            ImGui.inputInt("##port", port);
            ImGui.spacing();
            label("User:");
            ImGui.inputText("##username", username);
            label("Use password:");
            ImGui.checkbox("##use_password", hasPassword);
            if (!hasPassword.get()) {
                password.set("");
                showPassword.set(false);
            }
            label("Password:");
            ImGui.beginDisabled(!hasPassword.get());
            ImGui.inputText("##password", password, showPassword.get() ? 0 : ImGuiInputTextFlags.Password);
            ImGui.endDisabled();
            label("Show password:");
            ImGui.beginDisabled();
            ImGui.checkbox("##show_password", showPassword);
            ImGui.endDisabled();

            ImGui.endTable();
        }
    }
}
