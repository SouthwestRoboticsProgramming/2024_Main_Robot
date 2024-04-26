package com.swrobotics.blockauto.tool.sftp;

import java.util.concurrent.CompletableFuture;

public final class ProgressableFuture<T> extends CompletableFuture<T> {
    private long bytesSoFar;
    private long bytesTotal;

    public long getBytesSoFar() {
        return bytesSoFar;
    }

    public void setBytesSoFar(long bytesSoFar) {
        this.bytesSoFar = bytesSoFar;
    }

    public long getBytesTotal() {
        return bytesTotal;
    }

    public void setBytesTotal(long bytesTotal) {
        this.bytesTotal = bytesTotal;
    }
}
