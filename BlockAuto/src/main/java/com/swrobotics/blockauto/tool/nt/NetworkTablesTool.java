package com.swrobotics.blockauto.tool.nt;

import com.swrobotics.blockauto.tool.Tool;

import edu.wpi.first.networktables.NetworkTableInstance;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiTableFlags;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImInt;
import imgui.type.ImString;

import java.util.concurrent.ExecutorService;

public final class NetworkTablesTool implements Tool {
    private static final String TITLE = "NetworkTables";

    private static final int VERSION_NT3 = 0;
    private static final int VERSION_NT4 = 1;
    private static final String[] VERSION_NAMES = {"NT3 (Old)", "NT4 (New)"};

    private static final int CONN_MODE_TEAM_NUMBER = 0;
    private static final int CONN_MODE_ADDRESS = 1;
    private static final int CONN_MODE_SERVER = 2;
    private static final String[] CONN_MODE_NAMES = {"Team Number", "Address", "Server"};

    private static final int DEFAULT_VERSION = VERSION_NT4;
    private static final int DEFAULT_CONN_MODE = CONN_MODE_TEAM_NUMBER;
    private static final String DEFAULT_HOST = "localhost";
    private static final int[] DEFAULT_PORT_PER_VERSION = {
        NetworkTableInstance.kDefaultPort3, NetworkTableInstance.kDefaultPort4
    };
    private static final int DEFAULT_TEAM_NUMBER = 2129;

    private final ImInt version;
    private final ImInt connectionMode;
    private final ImString host;
    private final ImInt portOrTeamNumber;

    private final NetworkTablesConnection connection;

    public NetworkTablesTool(ExecutorService threadPool) {
        version = new ImInt(DEFAULT_VERSION);
        connectionMode = new ImInt(DEFAULT_CONN_MODE);

        host = new ImString(64);
        host.set(DEFAULT_HOST);
        portOrTeamNumber = new ImInt(getDefaultPortOrTeamNumber());

        connection = new NetworkTablesConnection(threadPool);
    }

    public void addListener(NTInstanceListener listener) {
        connection.addListener(listener);
    }

    // --- Server connection ---

    private int getDefaultPortOrTeamNumber() {
        if (connectionMode.get() == CONN_MODE_TEAM_NUMBER) return DEFAULT_TEAM_NUMBER;

        return DEFAULT_PORT_PER_VERSION[version.get()];
    }

    private void updateConnectionServer() {
        NetworkTablesConnection.Params params;
        if (connectionMode.get() == CONN_MODE_TEAM_NUMBER)
            params =
                    new NetworkTablesConnection.Params(
                            NetworkTablesConnection.ConnectionMode.CLIENT_TEAM_NUMBER,
                            null,
                            portOrTeamNumber.get());
        else if (connectionMode.get() == CONN_MODE_ADDRESS)
            params =
                    new NetworkTablesConnection.Params(
                            NetworkTablesConnection.ConnectionMode.CLIENT_ADDRESS,
                            host.get(),
                            portOrTeamNumber.get());
        else
            params =
                    new NetworkTablesConnection.Params(
                            NetworkTablesConnection.ConnectionMode.SERVER, null, 0);

        connection.setServerParams(version.get() == VERSION_NT4, params);
    }

    private void label(String label) {
        ImGui.tableNextColumn();
        ImGui.alignTextToFramePadding();
        ImGui.text(label);
        ImGui.tableNextColumn();
        ImGui.setNextItemWidth(-1);
    }

    private void showConnectionInfo() {
        if (ImGui.beginTable("layout", 2, ImGuiTableFlags.SizingStretchProp)) {
            label("NT Version:"); ImGui.combo("##version", version, VERSION_NAMES);

            label("Connection Mode:");
            boolean connModeChanged = ImGui.combo("##conn_mode", connectionMode, CONN_MODE_NAMES);
            if (connModeChanged) portOrTeamNumber.set(getDefaultPortOrTeamNumber());

            if (connectionMode.get() == CONN_MODE_TEAM_NUMBER) {
                label("Team Number:");
                ImGui.inputInt("##team_num", portOrTeamNumber);
            } else if (connectionMode.get() == CONN_MODE_ADDRESS) {
                label("Host:");
                ImGui.inputText("##host", host);
                label("Port:");
                ImGui.inputInt("##port", portOrTeamNumber);
            }

            ImGui.tableNextColumn();
            ImGui.text("Status");
            ImGui.tableNextColumn();
            NetworkTablesConnection.Status status = connection.getStatus();
            ImGui.pushStyleColor(ImGuiCol.Text, status.getColor());
            ImGui.text(status.getFriendlyName());
            ImGui.popStyleColor();

            ImGui.endTable();
        }

        updateConnectionServer();
    }

    @Override
    public void process() {
        if (ImGui.begin(TITLE, ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoScrollWithMouse)) {
            showConnectionInfo();
        }
        ImGui.end();
    }
}
