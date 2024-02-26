package com.swrobotics.shufflelog.tool.sftp;

public final class RemoteFile implements RemoteEntry {
    private final String fileName;

    public RemoteFile(String fileName) {
        this.fileName = fileName;
    }

    @Override
    public String getFileName() {
        return fileName;
    }
}
