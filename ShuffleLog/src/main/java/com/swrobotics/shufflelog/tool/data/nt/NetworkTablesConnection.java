package com.swrobotics.shufflelog.tool.data.nt;

import com.swrobotics.shufflelog.tool.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTableInstance;

import imgui.ImGui;

import java.util.HashSet;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;

public final class NetworkTablesConnection {
    public enum ConnectionMode {
        CLIENT_ADDRESS,
        CLIENT_TEAM_NUMBER,
        SERVER;

        public boolean isClient() {
            return this != SERVER;
        }
    }

    public static final class Params {
        private final ConnectionMode mode;
        private final String host;
        private final int portOrTeam;

        public Params(ConnectionMode mode, String host, int portOrTeam) {
            this.mode = mode;
            this.host = host;
            this.portOrTeam = portOrTeam;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            Params params = (Params) o;
            return mode == params.mode
                    && portOrTeam == params.portOrTeam
                    && Objects.equals(host, params.host);
        }

        @Override
        public int hashCode() {
            return Objects.hash(mode, host, portOrTeam);
        }
    }

    public enum Status {
        IDLE("Not Connected", 1, 0, 0),
        CONNECTED("Connected", 0, 1, 0),
        CLOSING("Switching", 1, 0.5f, 0);

        private final String friendlyName;
        private final int color;

        Status(String friendlyName, float r, float g, float b) {
            this.friendlyName = friendlyName;
            this.color = ImGui.colorConvertFloat4ToU32(r, g, b, 1);
        }

        public String getFriendlyName() {
            return friendlyName;
        }

        public int getColor() {
            return color;
        }
    }

    private static final String CLIENT_ID = "ShuffleLog";

    private final ExecutorService threadPool;
    private final Set<NTInstanceListener> listeners;
//    private final SmartDashboard smartDashboard;

    private NetworkTableInstance instance;
    private Future<?> stopFuture;
    private Boolean isNt4;
    private Params params;

    private NetworkTableRepr rootTable;
    private final AtomicInteger activeInstances;

    public NetworkTablesConnection(ExecutorService threadPool) {
        this.threadPool = threadPool;
        listeners = new HashSet<>();

        instance = null;
        stopFuture = null;
        isNt4 = null;
        params = null;

        activeInstances = new AtomicInteger(0);
    }

    public void setServerParams(boolean isNt4, Params params) {
        // Start a new client if there is not one currently
        if (instance == null) {
            instance = NetworkTableInstance.create();
            activeInstances.incrementAndGet();

            if (params.mode.isClient()) {
                if (isNt4) instance.startClient4(CLIENT_ID);
                else instance.startClient3(CLIENT_ID);

                if (params.mode == ConnectionMode.CLIENT_ADDRESS) instance.setServer(params.host, params.portOrTeam);
                else instance.setServerTeam(params.portOrTeam);
            } else {
                instance.startServer("shufflelog-nt.ini");
            }

            this.isNt4 = isNt4;
            rootTable = new NetworkTableRepr(instance.getTable("/"));
            for (NTInstanceListener listener : listeners)
                listener.onNTInit(instance);

            this.params = params;
        }

        // If the current client already satisfies the desired parameters,
        // it doesn't need to restart
        if (this.isNt4 == isNt4 && params.equals(this.params)) return;

        // Wait for the stop future to finish so we don't create
        // too many instances (there is a maximum of 16)
        if (stopFuture != null && !stopFuture.isDone()) return;

        // Dispose of cached entries if they exist
        if (rootTable != null) rootTable.close();
        rootTable = null;

        // Save a reference to the current instance and clear instance variable so
        // the instance can't be used after closing
        NetworkTableInstance savedInstance = instance;
        instance = null;
        boolean savedIsClient = params.mode.isClient();
        for (NTInstanceListener listener : listeners)
            listener.onNTClose();

        // Stop on other thread because stopping the client can take around
        // 10 seconds sometimes, and we don't want to freeze the GUI
        // We need to completely restart the NT instance because NT does not clear
        // local entries when switching servers
        stopFuture =
                threadPool.submit(
                        () -> {
                            if (savedIsClient) {
                                savedInstance.setServer(new String[0], 0);
                                savedInstance.stopClient();
                            } else {
                                savedInstance.stopServer();
                            }
                            savedInstance.stopLocal();
                            savedInstance.close();
                            activeInstances.decrementAndGet();
                        });
    }

    public Status getStatus() {
        if (stopFuture != null && !stopFuture.isDone()) return Status.CLOSING;

        if (params != null && params.mode == ConnectionMode.SERVER)
            return Status.CONNECTED;

        return (instance != null && instance.isConnected()) ? Status.CONNECTED : Status.IDLE;
    }

    public void addListener(NTInstanceListener listener) {
        listeners.add(listener);
    }

    // Can be null if client is not running
    public NetworkTableRepr getRootTable() {
        return rootTable;
    }

    public NetworkTableInstance getInstance() {
        return instance;
    }

    public int getActiveInstances() {
        return activeInstances.get();
    }
}
