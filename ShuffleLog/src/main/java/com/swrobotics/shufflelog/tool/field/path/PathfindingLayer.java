package com.swrobotics.shufflelog.tool.field.path;

import com.google.gson.JsonObject;
import com.swrobotics.messenger.client.MessageBuilder;
import com.swrobotics.messenger.client.MessageReader;
import com.swrobotics.messenger.client.MessengerClient;
import com.swrobotics.shufflelog.json.JsonObj;
import com.swrobotics.shufflelog.tool.ToolConstants;
import com.swrobotics.shufflelog.tool.field.FieldLayer;
import com.swrobotics.shufflelog.tool.field.path.shape.Circle;
import com.swrobotics.shufflelog.tool.field.path.shape.Rectangle;
import com.swrobotics.shufflelog.tool.field.path.shape.Shape;
import com.swrobotics.shufflelog.util.Cooldown;
import com.swrobotics.shufflelog.util.ExpressionInput;
import imgui.ImGui;
import imgui.flag.ImGuiTableFlags;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.type.ImBoolean;
import processing.core.PConstants;
import processing.core.PGraphics;

import java.util.*;

public final class PathfindingLayer implements FieldLayer {
    // Main API
    private static final String MSG_SET_POS = "Pathfinder:SetPos";
    private static final String MSG_SET_GOAL = "Pathfinder:SetGoal";
    private static final String MSG_PATH = "Pathfinder:Path";

    // ShuffleLog API
    private static final String MSG_GET_FIELD_INFO = "Pathfinder:GetFieldInfo";
    private static final String MSG_GET_CELL_DATA = "Pathfinder:GetCellData";
    private static final String MSG_GET_SHAPES = "Pathfinder:GetShapes";

    private static final String MSG_SET_SHAPE = "Pathfinder:SetShape";
    private static final String MSG_REMOVE_SHAPE = "Pathfinder:RemoveShape";
    private static final String MSG_SET_DYN_SHAPES = "Pathfinder:SetDynamicShapes";

    private static final String MSG_FIELD_INFO = "Pathfinder:FieldInfo";
    private static final String MSG_CELL_DATA = "Pathfinder:CellData";
    private static final String MSG_SHAPES = "Pathfinder:Shapes";

    private final MessengerClient msg;
    private final Cooldown reqFieldInfoCooldown;
    private final Cooldown reqGridsCooldown;
    private final Cooldown reqCellDataCooldown;

    private final ImBoolean showGridLines;
    private final ImBoolean showGridCells;
    private final ImBoolean showShapes;
    private final ImBoolean showDynShapes;
    private final ImBoolean showPath;

    private FieldInfo fieldInfo;
    private List<Point> path;
    private Grid2D cellData;
    private boolean needsRefreshCellData;
    private List<Shape> shapes, dynShapes;

    private double startX, startY;
    private double goalX, goalY;

    private Shape hoveredShape;

    public PathfindingLayer(MessengerClient msg) {
        this.msg = msg;
        reqFieldInfoCooldown = new Cooldown(ToolConstants.MSG_QUERY_COOLDOWN_TIME);
        reqGridsCooldown = new Cooldown(ToolConstants.MSG_QUERY_COOLDOWN_TIME);
        reqCellDataCooldown = new Cooldown(ToolConstants.MSG_QUERY_COOLDOWN_TIME);

        msg.addHandler(MSG_PATH, this::onPath);
        msg.addHandler(MSG_FIELD_INFO, this::onFieldInfo);
        msg.addHandler(MSG_SHAPES, this::onShapes);
        msg.addHandler(MSG_CELL_DATA, this::onCellData);
        msg.addHandler(MSG_SET_POS, this::onSetPos);
        msg.addHandler(MSG_SET_GOAL, this::onSetGoal);
        msg.addHandler(MSG_SET_DYN_SHAPES, this::onSetDynShapes);

        msg.addDisconnectHandler(
                () -> {
                    fieldInfo = null;
                    path = null;
                    shapes = null;
                    cellData = null;
                    needsRefreshCellData = true;
                });

        showGridLines = new ImBoolean(false);
        showGridCells = new ImBoolean(true);
        showShapes = new ImBoolean(true);
        showDynShapes = new ImBoolean(true);
        showPath = new ImBoolean(true);

        fieldInfo = null;
        path = null;
        shapes = null;
        cellData = null;
        needsRefreshCellData = true;
        dynShapes = new ArrayList<>();
    }

    private void onPath(String type, MessageReader reader) {
        boolean valid = reader.readBoolean();
        if (valid) {
            if (path != null) path.clear();
            else path = new ArrayList<>();

            int count = reader.readInt();
            for (int i = 0; i < count; i++) {
                double x = reader.readDouble();
                double y = reader.readDouble();
                path.add(new Point(x, y));
            }
        } else {
            path = null;
        }
    }

    private void onFieldInfo(String type, MessageReader reader) {
        fieldInfo = new FieldInfo(reader);
    }

    private void onShapes(String type, MessageReader reader) {
        int count = reader.readInt();
        shapes = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            Shape shape = Shape.read(reader, true);
            shapes.add(shape);
        }

        onSetDynShapes(null, reader);
    }

    private void onCellData(String type, MessageReader reader) {
        cellData = new Grid2D();
        cellData.readContent(reader);
        needsRefreshCellData = false;
    }

    private void onSetPos(String type, MessageReader reader) {
        startX = reader.readDouble();
        startY = reader.readDouble();
    }

    private void onSetGoal(String type, MessageReader reader) {
        goalX = reader.readDouble();
        goalY = reader.readDouble();
    }

    private void onSetDynShapes(String type, MessageReader reader) {
        dynShapes.clear();
        int count = reader.readInt();
        for (int i = 0; i < count; i++) {
            Shape shape = Shape.read(reader, false);
            dynShapes.add(shape);
        }
        needsRefreshCellData = true;
    }

    @Override
    public String getName() {
        return "Pathfinding";
    }

    @Override
    public void draw(PGraphics g) {
        if (!msg.isConnected()) return;

        if (shapes == null && reqGridsCooldown.request()) {
            msg.send(MSG_GET_SHAPES);
        }
        if (needsRefreshCellData && reqCellDataCooldown.request()) {
            msg.send(MSG_GET_CELL_DATA);
        }
        if (fieldInfo == null) {
            if (reqFieldInfoCooldown.request()) msg.send(MSG_GET_FIELD_INFO);
            return;
        }

        boolean lines = showGridLines.get();
        boolean cells = showGridCells.get();
        boolean shapes = showShapes.get();
        boolean dynShapes = showDynShapes.get();
        boolean path = showPath.get();

        g.pushMatrix();
        {
            // Transform into cell space
            float cellSize = (float) fieldInfo.getCellSize();
            g.scale(cellSize, -cellSize);
            g.translate((float) -fieldInfo.getOriginX(), (float) -fieldInfo.getOriginY());
            float cellStrokeMul = 1 / cellSize;

            int cellsX = fieldInfo.getCellsX();
            int cellsY = fieldInfo.getCellsY();

            // Show cell data content
            if (cells && cellData != null) {
                g.fill(200, 0, 0, 196);
                g.noStroke();
                for (int y = 0; y < cellsY; y++) {
                    for (int x = 0; x < cellsX; x++) {
                        boolean passable = cellData.get(x, y);
                        if (!passable) {
                            g.rect(x, y, 1, 1);
                        }
                    }
                }
            }

            // Show grid lines
            if (lines) {
                g.strokeWeight(0.5f * cellStrokeMul);
                g.stroke(96);

                for (int x = 0; x <= cellsX; x++) {
                    g.line(x, 0, x, cellsY);
                }
                for (int y = 0; y <= cellsY; y++) {
                    g.line(0, y, cellsX, y);
                }
            }
        }
        g.popMatrix();

        // Show shapes
        if (shapes && this.shapes != null) {
            for (Shape shape : this.shapes) {
                drawShape(g, shape, g.color(201, 101, 18), g.color(201, 101, 18, 128));
            }
            if (hoveredShape != null)
                drawShape(g, hoveredShape, g.color(46, 174, 217), g.color(46, 174, 217, 128));
        }

        if (dynShapes && this.shapes != null) {
            for (Shape shape : this.dynShapes) {
                drawShape(g, shape, g.color(191, 66, 245), g.color(191, 66, 245, 128));
            }
        }

        // Show path
        if (path) {
            if (this.path != null) {
                g.strokeWeight(4);
                g.stroke(214, 196, 32, 128);
                g.beginShape(PConstants.LINE_STRIP);
                for (Point p : this.path) g.vertex((float) p.x, (float) p.y);
                g.endShape();

                g.strokeWeight(2);
                g.stroke(214, 196, 32);
                g.beginShape(PConstants.LINE_STRIP);
                for (Point p : this.path) g.vertex((float) p.x, (float) p.y);
                g.endShape();
            }

            // Show endpoints
            g.pushMatrix();
            g.translate(0, 0, 0.005f);
            g.strokeWeight(1);
            g.ellipseMode(PConstants.CENTER);
            g.stroke(27, 196, 101, 128);
            g.fill(27, 196, 101);
            float startSize = startX == goalX && startY == goalY ? 0.12f : 0.10f;
            g.ellipse((float) startX, (float) startY, startSize, startSize);
            g.stroke(44, 62, 199, 128);
            g.fill(44, 62, 199);
            g.ellipse((float) goalX, (float) goalY, 0.10f, 0.10f);
            g.popMatrix();
        }
    }

    private void drawShape(PGraphics g, Shape shape, int fg, int bg) {
        if (shape instanceof Circle c) {
            g.ellipseMode(PConstants.CENTER);
            g.noFill();

            float x = (float) c.x.get();
            float y = (float) c.y.get();
            float d = (float) (2 * c.radius.get());

            g.strokeWeight(4);
            g.stroke(bg);
            g.ellipse(x, y, d, d);
            g.strokeWeight(2);
            g.stroke(fg);
            g.ellipse(x, y, d, d);
        } else if (shape instanceof Rectangle r) {
            float x = (float) r.x.get();
            float y = (float) r.y.get();
            float w = (float) r.width.get();
            float h = (float) r.height.get();
            float rot = (float) r.rotation.get();

            g.pushMatrix();
            g.translate(x, y);
            g.rotate((float) Math.toRadians(rot));

            g.noFill();
            g.strokeWeight(4);
            g.stroke(bg);
            g.rect(-w / 2, -h / 2, w, h);
            g.stroke(fg);
            g.strokeWeight(2);
            g.rect(-w / 2, -h / 2, w, h);

            g.popMatrix();
        }
    }

    private void removeShape(Shape shape) {
        shapes.remove(shape);
        msg.prepare(MSG_REMOVE_SHAPE)
                .addLong(shape.getId().getMostSignificantBits())
                .addLong(shape.getId().getLeastSignificantBits())
                .send();
        needsRefreshCellData = true;
    }

    private void fieldHeader(String name) {
        ImGui.tableNextColumn();
        ImGui.alignTextToFramePadding();
        ImGui.treeNodeEx(
                name,
                ImGuiTreeNodeFlags.Leaf
                        | ImGuiTreeNodeFlags.NoTreePushOnOpen
                        | ImGuiTreeNodeFlags.SpanFullWidth);
        ImGui.tableNextColumn();
        ImGui.setNextItemWidth(-1);
    }

    private void showCircle(Circle circle) {
        String id = "Circle##" + circle.getId();

        ImGui.tableNextColumn();
        boolean open = ImGui.treeNodeEx(id, ImGuiTreeNodeFlags.SpanFullWidth);
        if (ImGui.isItemHovered()) hoveredShape = circle;
        if (ImGui.beginPopupContextItem()) {
            if (ImGui.selectable("Delete")) {
                removeShape(circle);
            }
            ImGui.endPopup();
        }
        ImGui.tableNextColumn();
        ImGui.textDisabled(circle.getId().toString());

        if (open) {
            boolean changed;

            fieldHeader("X");
            changed = ExpressionInput.inputDouble("##x", circle.x);
            fieldHeader("Y");
            changed |= ExpressionInput.inputDouble("##y", circle.y);
            fieldHeader("Radius");
            changed |= ExpressionInput.inputDouble("##radius", circle.radius);

            if (changed) {
                MessageBuilder builder = msg.prepare(MSG_SET_SHAPE);
                circle.write(builder);
                builder.send();
                needsRefreshCellData = true;
            }

            ImGui.treePop();
        }
    }

    private void showRectangle(Rectangle rect) {
        String id = "Rectangle##" + rect.getId();

        ImGui.tableNextColumn();
        boolean open = ImGui.treeNodeEx(id, ImGuiTreeNodeFlags.SpanFullWidth);
        if (ImGui.isItemHovered()) hoveredShape = rect;
        if (ImGui.beginPopupContextItem()) {
            if (ImGui.selectable("Delete")) {
                removeShape(rect);
            }
            ImGui.endPopup();
        }
        ImGui.tableNextColumn();
        ImGui.textDisabled(rect.getId().toString());

        if (open) {
            boolean changed;
            fieldHeader("X");
            changed = ExpressionInput.inputDouble("##x", rect.x);
            fieldHeader("Y");
            changed |= ExpressionInput.inputDouble("##y", rect.y);
            fieldHeader("Width");
            changed |= ExpressionInput.inputDouble("##width", rect.width);
            fieldHeader("Height");
            changed |= ExpressionInput.inputDouble("##height", rect.height);
            fieldHeader("Rotation");
            changed |= ExpressionInput.inputDouble("##rotation", rect.rotation);
            fieldHeader("Inverted");
            changed |= ImGui.checkbox("##inverted", rect.inverted);

            if (changed) {
                MessageBuilder builder = msg.prepare(MSG_SET_SHAPE);
                rect.write(builder);
                builder.send();
                needsRefreshCellData = true;
            }

            ImGui.treePop();
        }
    }

    private void showShape(Shape shape) {
        if (shape instanceof Circle) showCircle((Circle) shape);
        else if (shape instanceof Rectangle) showRectangle((Rectangle) shape);
    }

    @Override
    public void showGui() {
        ImGui.checkbox("Show grid lines", showGridLines);
        ImGui.checkbox("Show grid cells", showGridCells);
        ImGui.checkbox("Show shapes", showShapes);
        ImGui.checkbox("Show dynamic shapes", showDynShapes);
        ImGui.checkbox("Show path", showPath);
        ImGui.separator();
        if (!msg.isConnected()) {
            ImGui.textDisabled("Not connected");
            return;
        }
        if (shapes != null) {
            if (ImGui.beginTable("shapes", 2, ImGuiTableFlags.Borders | ImGuiTableFlags.Resizable)) {
                hoveredShape = null;

                ImGui.tableNextColumn();
                boolean open = ImGui.treeNodeEx("Shapes", ImGuiTreeNodeFlags.DefaultOpen);
                if (ImGui.beginPopupContextItem()) {
                    Shape addedShape = null;
                    if (ImGui.selectable("Add Circle")) {
                        Circle c = new Circle(UUID.randomUUID());
                        c.x.set(0);
                        c.y.set(0);
                        c.radius.set(1);
                        addedShape = c;
                    }
                    if (ImGui.selectable("Add Rectangle")) {
                        Rectangle r = new Rectangle(UUID.randomUUID());
                        r.x.set(0);
                        r.y.set(0);
                        r.width.set(1);
                        r.height.set(1);
                        r.rotation.set(0);
                        r.inverted.set(false);
                        addedShape = r;
                    }

                    if (addedShape != null) {
                        shapes.add(addedShape);
                        MessageBuilder builder = msg.prepare(MSG_SET_SHAPE);
                        addedShape.write(builder);
                        builder.send();
                        needsRefreshCellData = true;
                    }

                    ImGui.endPopup();
                }
                ImGui.tableNextColumn();
                if (open) {
                    for (Shape shape : new ArrayList<>(shapes)) {
                        showShape(shape);
                    }
                    ImGui.treePop();
                }

                ImGui.endTable();

                if (ImGui.treeNodeEx("Dynamic Shapes", ImGuiTreeNodeFlags.DefaultOpen)) {
                    int i = 0;
                    for (Shape shape : dynShapes) {
                        if (shape instanceof Rectangle r) {
                            if (ImGui.treeNodeEx("Rectangle##" + i++)) {
                                ImGui.text("X: " + r.x.get());
                                ImGui.text("Y: " + r.y.get());
                                ImGui.text("Width: " + r.width.get());
                                ImGui.text("Height: " + r.height.get());
                                ImGui.text("Rotation: " + r.rotation.get());
                                ImGui.text("Inverted: " + r.inverted.get());
                                ImGui.treePop();
                            }
                        } else if (shape instanceof Circle c) {
                            if (ImGui.treeNodeEx("Circle##" + i++)) {
                                ImGui.text("X: " + c.x.get());
                                ImGui.text("Y: " + c.y.get());
                                ImGui.text("Radius: " + c.radius.get());
                                ImGui.treePop();
                            }
                        }
                    }
                    ImGui.treePop();
                }
            }
        }
    }

    /*
    private final ImBoolean showGridLines;
    private final ImBoolean showGridCells;
    private final ImBoolean showShapes;
    private final ImBoolean showDynShapes;
    private final ImBoolean showPath;
     */

    @Override
    public void load(JsonObj obj) {
        JsonObj o = obj.getObject("pathfinding");
        showGridLines.set(o.getBoolean("gridLines", false));
        showGridCells.set(o.getBoolean("gridCells", true));
        showShapes.set(o.getBoolean("shapes", true));
        showDynShapes.set(o.getBoolean("dynShapes", true));
        showPath.set(o.getBoolean("path", true));
    }

    @Override
    public void store(JsonObject obj) {
        JsonObject o = new JsonObject();
        o.addProperty("gridLines", showGridLines.get());
        o.addProperty("gridCells", showGridCells.get());
        o.addProperty("shapes", showShapes.get());
        o.addProperty("dynShapes", showDynShapes.get());
        o.addProperty("path", showPath.get());
        obj.add("pathfinding", o);
    }
}
