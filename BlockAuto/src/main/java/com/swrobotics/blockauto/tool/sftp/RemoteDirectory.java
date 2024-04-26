package com.swrobotics.blockauto.tool.sftp;

import java.util.List;

public final class RemoteDirectory implements RemoteEntry {
    private final String fileName;
    private final String path;
    private List<RemoteEntry> children;

    public RemoteDirectory(String fileName, String path) {
        this.fileName = fileName;
        this.path = path;
        children = null;
    }

    @Override
    public String getFileName() {
        return fileName;
    }

    public String getPath() {
        return path;
    }

    public List<RemoteEntry> getChildren() {
        return children;
    }

    public void setChildren(List<RemoteEntry> children) {
        this.children = children;
    }
}
