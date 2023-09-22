package com.swrobotics.shufflelog.tool.smartdashboard;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.swrobotics.shufflelog.json.JsonArr;
import com.swrobotics.shufflelog.json.JsonObj;
import com.swrobotics.shufflelog.render.ImGuiRenderer;
import com.swrobotics.shufflelog.render.Renderer2d;
import com.swrobotics.shufflelog.tool.Tool;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import imgui.ImGui;
import imgui.ImVec2;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public final class SmartDashboard implements Tool {
    private NetworkTable table;
    private Set<String> openItems;

    public SmartDashboard() {
        table = null;
        openItems = new HashSet<>();
    }

    public void init(NetworkTableInstance instance) {
        table = instance.getTable("SmartDashboard");
    }

    public void close() {
        table = null;
    }

    private void showMechanism2dWindow(Mechanism2dView view) {
        ImVec2 pos = ImGui.getWindowPos();
        ImVec2 min = ImGui.getWindowContentRegionMin();
        ImVec2 max = ImGui.getWindowContentRegionMax();

        double w = max.x - min.x;
        double h = max.y - min.y;
        double scaleX = w / view.width;
        double scaleY = h / view.height;
        double scale = Math.min(scaleX, scaleY);

        Renderer2d renderer = new ImGuiRenderer(ImGui.getWindowDrawList());
        renderer.translate(pos.x + (min.x + max.x) / 2, pos.y + (min.y + max.y) / 2); // To middle
        renderer.scale(scale, -scale);
        renderer.translate(-view.width / 2, -view.height / 2);

        view.render(renderer);
    }

    @Override
    public void process() {
        for (String open : openItems) {
            if (ImGui.begin("SD: " + open)) {
                if (table == null) {
                    ImGui.textDisabled("No NT instance");
                    ImGui.end();
                    continue;
                }
                if (!table.containsSubTable(open)) {
                    ImGui.textDisabled("Not currently published");
                    ImGui.end();
                    continue;
                }

                NetworkTable subTable = table.getSubTable(open);
                String type = subTable.getEntry(".type").getString("unknown");
                switch (type) {
                    case "Mechanism2d": showMechanism2dWindow(new Mechanism2dView(subTable)); break;
                    default:
                        ImGui.textDisabled("Unknown item type: " + type);
                }
            }
            ImGui.end();
        }
    }

    public void showMenu() {
        if (ImGui.beginMenu("SmartDashboard")) {
            Set<String> avail = table.getSubTables();
            List<String> all = new ArrayList<>(avail);
            for (String name : openItems) {
                if (!all.contains(name))
                    all.add(name);
            }
            all.sort(String.CASE_INSENSITIVE_ORDER);

            for (String name : all) {
                boolean enabled = openItems.contains(name);
                if (ImGui.menuItem(name, null, enabled)) {
                    if (enabled)
                        openItems.remove(name);
                    else
                        openItems.add(name);
                }
            }

            ImGui.endMenu();
        }
    }

    @Override
    public void load(JsonObj obj) {
        JsonObj config = obj.getObject("smartdashboard");
        JsonArr openItems = config.getArray("open");
        for (JsonElement elem : openItems) {
            this.openItems.add(elem.getAsString());
        }
    }

    @Override
    public void store(JsonObject obj) {
        JsonObject config = new JsonObject();
        JsonArray openItems = new JsonArray();
        for (String item : this.openItems) {
            openItems.add(item);
        }
        config.add("open", openItems);
        obj.add("smartdashboard", config);
    }
}
