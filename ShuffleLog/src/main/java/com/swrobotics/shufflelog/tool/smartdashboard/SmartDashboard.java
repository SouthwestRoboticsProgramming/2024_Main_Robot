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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.type.ImBoolean;
import imgui.type.ImInt;

import java.util.*;

public final class SmartDashboard implements Tool {
    private NetworkTable table;
    private final Set<String> openItems;
    private final Map<String, Field2dSettings> field2dSettings;
    private final ImBoolean closeButton;
    private final ImInt chooserIdx;

    public SmartDashboard() {
        table = null;
        openItems = new HashSet<>();
        field2dSettings = new HashMap<>();
        closeButton = new ImBoolean();
        chooserIdx = new ImInt();
    }

    public void init(NetworkTableInstance instance) {
        table = instance.getTable("SmartDashboard");
    }

    public void close() {
        table = null;
    }

    private Field2dSettings getField2dSettings(String name) {
        return field2dSettings.computeIfAbsent(name, (n) -> new Field2dSettings(new JsonObj(null)));
    }

    public List<Field2dInfo> getAllField2d() {
        if (table == null) return Collections.emptyList();

        Set<String> avail = table.getSubTables();
        List<Field2dInfo> out = new ArrayList<>();
        for (String name : avail) {
            NetworkTable subTable = table.getSubTable(name);
            if (subTable.getEntry(".type").getString("unknown").equals("Field2d")) {
                out.add(new Field2dInfo(name, getField2dSettings(name), new Field2dView(subTable)));
            }
        }

        return out;
    }

    private Renderer2d createScaledRenderer(double width, double height) {
        ImVec2 pos = ImGui.getWindowPos();
        ImVec2 min = ImGui.getWindowContentRegionMin();
        ImVec2 max = ImGui.getWindowContentRegionMax();

        double w = max.x - min.x;
        double h = max.y - min.y;
        double scaleX = w / width;
        double scaleY = h / height;
        double scale = Math.min(scaleX, scaleY);

        Renderer2d renderer = new ImGuiRenderer(ImGui.getWindowDrawList());
        renderer.translate(pos.x + (min.x + max.x) / 2, pos.y + (min.y + max.y) / 2); // To middle
        renderer.scale(scale, -scale);
        renderer.translate(-width / 2, -height / 2);

        return renderer;
    }

    private void showMechanism2dWindow(Mechanism2dView view) {
        view.render(createScaledRenderer(view.width, view.height));
    }

    private void showField2dWindow(String name, Field2dView view) {
        Field2dSettings settings = getField2dSettings(name);
        Set<String> published = view.poseSets.keySet();
        settings.update(published);

        if (ImGui.beginPopupContextWindow()) {
            settings.edit(published);
            ImGui.endPopup();
        }

        view.render(
                createScaledRenderer(settings.fieldWidth.get(), settings.fieldHeight.get()),
                settings,
                true);
    }

    private void showChooserWindow(NetworkTable table) {
        NetworkTableEntry selectedEntry = table.getEntry("selected");

        String current = table.getEntry("active").getString("");
        String selected = selectedEntry.getString(current);
        String[] options = table.getEntry("options").getStringArray(new String[0]);

        boolean found = false;
        for (int i = 0; i < options.length; i++) {
            if (options[i].equals(selected)) {
                chooserIdx.set(i);
                found = true;
                break;
            }
        }
        if (!found) {
            ImGui.textDisabled("Selection not in options?? WPILib bug");
            return;
        }

        String acceptedByRobot = current.equals(selected) ? "GOOD" : "WAITING";
        if (ImGui.combo(acceptedByRobot + "##chooser", chooserIdx, options)) {
            // Keep the entry there if ShuffleLog disconnects
            selectedEntry.getTopic().setRetained(true);
            selectedEntry.setString(options[chooserIdx.get()]);
        }
    }

    @Override
    public void process() {
        for (Iterator<String> iter = openItems.iterator(); iter.hasNext(); ) {
            String open = iter.next();

            closeButton.set(true);
            if (ImGui.begin("SD: " + open, closeButton)) {
                if (table == null) {
                    ImGui.textDisabled("No NT instance");
                    ImGui.end();
                    if (!closeButton.get()) iter.remove();
                    continue;
                }
                if (!table.containsSubTable(open)) {
                    ImGui.textDisabled("Not currently published");
                    ImGui.end();
                    if (!closeButton.get()) iter.remove();
                    continue;
                }

                NetworkTable subTable = table.getSubTable(open);
                String type = subTable.getEntry(".type").getString("unknown");
                switch (type) {
                    case "Mechanism2d":
                        showMechanism2dWindow(new Mechanism2dView(subTable));
                        break;
                    case "Field2d":
                        showField2dWindow(open, new Field2dView(subTable));
                        break;
                    case "String Chooser":
                        showChooserWindow(subTable);
                        break;
                    case "unknown":
                        ImGui.textDisabled("Not currently published");
                        break;
                    default:
                        ImGui.textDisabled("Unknown item type: " + type);
                        break;
                }
            }
            ImGui.end();
            if (!closeButton.get()) iter.remove();
        }
    }

    public void showMenu() {
        if (ImGui.beginMenu("SmartDashboard")) {
            Set<String> avail = table.getSubTables();
            List<String> all = new ArrayList<>(avail);
            for (String name : openItems) {
                if (!all.contains(name)) all.add(name);
            }
            all.sort(String.CASE_INSENSITIVE_ORDER);

            for (String name : all) {
                boolean enabled = openItems.contains(name);
                if (ImGui.menuItem(name, null, enabled)) {
                    if (enabled) openItems.remove(name);
                    else openItems.add(name);
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
        JsonObj fieldSettings = config.getObject("fields");
        for (String key : fieldSettings.keySet()) {
            this.field2dSettings.put(key, new Field2dSettings(fieldSettings.getObject(key)));
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

        JsonObject fieldSettings = new JsonObject();
        for (Map.Entry<String, Field2dSettings> entry : field2dSettings.entrySet()) {
            fieldSettings.add(entry.getKey(), entry.getValue().save());
        }
        config.add("fields", fieldSettings);

        obj.add("smartdashboard", config);
    }
}
