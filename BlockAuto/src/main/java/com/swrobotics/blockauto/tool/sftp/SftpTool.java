package com.swrobotics.blockauto.tool.sftp;

import com.google.gson.JsonObject;
import com.jcraft.jsch.ChannelSftp;
import com.swrobotics.blockauto.StreamUtil;
import com.swrobotics.blockauto.json.JsonObj;
import com.swrobotics.blockauto.tool.Tool;
import com.swrobotics.blockauto.util.FileChooser;
import com.swrobotics.blockauto.util.FutureUtils;
import imgui.ImGui;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.type.ImBoolean;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;

public final class SftpTool implements Tool {
    private final ExecutorService threadPool;
    private final UUID uuid = UUID.randomUUID();
    private final SftpParameters params = new SftpParameters();
    private final ImBoolean showDotFiles = new ImBoolean(false);

    public SftpTool(ExecutorService threadPool) {
        this.threadPool = threadPool;
    }

    private CompletableFuture<SftpConnection> connectionFuture;
    private SftpConnection connection;
    private CompletableFuture<?> inProgressOperation;

    private RemoteDirectory rootDir;

    // TODO: State machine?
    @Override
    public void process() {
        if (ImGui.begin("SFTP: " + params.getName() + "##" + "a")) {
            ImGui.beginDisabled(isOperationInProgress());

            if (connection == null) {
                if (connectionFuture == null || connectionFuture.isCompletedExceptionally()) {
                    if (connectionFuture != null && connectionFuture.isCompletedExceptionally()) {
                        ImGui.text("Connection failed");
                        if (ImGui.isItemHovered()) {
                            ImGui.beginTooltip();
                            ImGui.text(StreamUtil.getStackTrace(FutureUtils.getException(connectionFuture)));
                            ImGui.endTooltip();
                        }
                        ImGui.separator();
                    }

                    params.edit();
                    if (ImGui.button("Connect")) {
                        connectionFuture = SftpConnection.connect(params, threadPool);
                    }
                } else if (connectionFuture.isDone()) {
                    connection = connectionFuture.getNow(null);
                    connectionFuture = null;
                    rootDir = new RemoteDirectory("", connection.getRemoteRootPath());
                } else {
                    ImGui.text("Connecting...");
                }
            }

            if (connection != null) {
                ImGui.checkbox("Show dot files", showDotFiles);
                if (ImGui.button("Disconnect")) {
                    inProgressOperation = connection.disconnect();
                    connection = null;
                }
            }

            if (connection == null) {
                ImGui.endDisabled();
                ImGui.end();
                return;
            }

            showDirectory(rootDir, true);

            ImGui.endDisabled();
        }
        ImGui.end();
    }

    private void showEntry(RemoteEntry entry) {
        if (entry instanceof RemoteFile)
            showFile((RemoteFile) entry);
        else
            showDirectory((RemoteDirectory) entry, false);
    }

    private void uploadFileInto(RemoteDirectory dir, File src) {
        
    }

    private void showDirectory(RemoteDirectory dir, boolean root) {
        int flags = root ? ImGuiTreeNodeFlags.DefaultOpen : 0;
        boolean open = ImGui.treeNodeEx(root ? "Root" : dir.getFileName(), flags);

        if (ImGui.beginPopupContextItem("context_menu")) {
            if (ImGui.selectable("Refresh")) {
                dir.setChildren(null);
                ImGui.closeCurrentPopup();
            }
            if (ImGui.selectable("Upload file(s)")) {
                ImGui.closeCurrentPopup();
                FileChooser.chooseFileOrFolder((file) -> uploadFileInto(dir, file));
            }
            ImGui.endPopup();
        }

        if (open) {
            List<RemoteEntry> children = dir.getChildren();
            if (children == null) {
                ImGui.textDisabled("Fetching...");

                // TODO: we could run multiple operations in parallel
                if (!isOperationInProgress()) {
                    CompletableFuture<List<ChannelSftp.LsEntry>> ls = connection.listDirectory(dir.getPath());
                    inProgressOperation = ls;

                    ls.thenAccept((entries) -> {
                        List<RemoteEntry> newChildren = new ArrayList<>();
                        for (ChannelSftp.LsEntry entry : entries) {
                            if (entry.getFilename().equals(".") || entry.getFilename().equals(".."))
                                continue;

                            if (entry.getAttrs().isDir()) {
                                String name = entry.getFilename();
                                newChildren.add(new RemoteDirectory(name, dir.getPath() + "/" + name));
                            } else {
                                newChildren.add(new RemoteFile(entry.getFilename()));
                            }
                        }
                        dir.setChildren(newChildren);
                    });
                }
            } else {
                children.sort(Comparator.comparing(RemoteEntry::getFileName));
                boolean dot = showDotFiles.get();
                for (RemoteEntry entry : children) {
                    if (!dot && entry.getFileName().startsWith("."))
                        continue;

                    showEntry(entry);
                }
            }
            ImGui.treePop();
        }
    }

    private void showFile(RemoteFile file) {
        ImGui.treeNodeEx(file.getFileName(), ImGuiTreeNodeFlags.Leaf | ImGuiTreeNodeFlags.NoTreePushOnOpen);
    }

    private boolean isOperationInProgress() {
        return inProgressOperation != null && !inProgressOperation.isDone();
    }

    @Override
    public void load(JsonObj obj) {
        params.load(obj.getObject("sftp"));
    }

    @Override
    public void store(JsonObject obj) {
        obj.add("sftp", params.save());
    }
}
