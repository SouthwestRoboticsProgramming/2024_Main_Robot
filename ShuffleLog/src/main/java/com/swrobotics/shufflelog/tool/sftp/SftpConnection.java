package com.swrobotics.shufflelog.tool.sftp;

import com.jcraft.jsch.*;
import com.swrobotics.shufflelog.StreamUtil;

import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;

public final class SftpConnection {
    private static final int SESSION_TIMEOUT = 10000;
    private static final int CHANNEL_TIMEOUT = 5000;

    public static CompletableFuture<SftpConnection> connect(SftpParameters params, ExecutorService threadPool) {
        CompletableFuture<SftpConnection> future = new CompletableFuture<>();
        threadPool.submit(() -> {
            JSch jsch = new JSch();
            Session session = null;
            try {
                session = jsch.getSession(params.getUsername(), params.getHost(), params.getPort());
                String password = params.getPassword();
                if (password != null)
                    session.setPassword(password);
                session.connect(SESSION_TIMEOUT);

                Channel rawChannel = session.openChannel("sftp");
                rawChannel.connect(CHANNEL_TIMEOUT);
                ChannelSftp channel = (ChannelSftp) rawChannel;

                future.complete(new SftpConnection(threadPool, session, channel));
            } catch (Throwable e) {
                if (session != null && session.isConnected()) {
                    session.disconnect();
                }
                future.completeExceptionally(e);
            }
        });
        return future;
    }

    private final ExecutorService threadPool;
    private final Session session;
    private final ChannelSftp channel;

    public SftpConnection(ExecutorService threadPool, Session session, ChannelSftp channel) {
        this.threadPool = threadPool;
        this.session = session;
        this.channel = channel;
    }

    private <T> CompletableFuture<T> runAsync(Callable<T> fn) {
        CompletableFuture<T> future = new CompletableFuture<>();
        threadPool.submit(() -> {
            try {
                future.complete(fn.call());
            } catch (Throwable e) {
                future.completeExceptionally(e);
            }
        });
        return future;
    }

    @SuppressWarnings("rawtypes")
    public CompletableFuture<List<ChannelSftp.LsEntry>> listDirectory(String remoteDirPath) {
        return runAsync(() -> {
            Vector vec;
            synchronized (channel) {
                vec = channel.ls(remoteDirPath);
            }

            List<ChannelSftp.LsEntry> list = new ArrayList<>();
            for (Object entry : vec) {
                list.add((ChannelSftp.LsEntry) entry);
            }
            return list;
        });
    }

    public ProgressableFuture<byte[]> readFile(String remoteFilePath) {
        ProgressableFuture<byte[]> future = new ProgressableFuture<>();
        threadPool.submit(() -> {
            try {
                byte[] data;
                synchronized (channel) {
                    InputStream in = channel.get(remoteFilePath, new ProgressUpdater(future));
                    data = StreamUtil.readToByteArray(in);
                }
                future.complete(data);
            } catch (Throwable t) {
                future.completeExceptionally(t);
            }
        });
        return future;
    }

    public ProgressableFuture<Void> writeFile(String remoteFilePath, byte[] data) {
        ProgressableFuture<Void> future = new ProgressableFuture<>();
        threadPool.submit(() -> {
            try {
                synchronized (channel) {
                    channel.put(new ByteArrayInputStream(data), remoteFilePath, new ProgressUpdater(future));
                }
                future.complete(null);
            } catch (Throwable t) {
                future.completeExceptionally(t);
            }
        });
        return future;
    }

    public CompletableFuture<Void> disconnect() {
        return runAsync(() -> {
            channel.exit();
            session.disconnect();
            return null;
        });
    }

    private static final class ProgressUpdater implements SftpProgressMonitor {
        private final ProgressableFuture<?> future;

        public ProgressUpdater(ProgressableFuture<?> future) {
            this.future = future;
        }

        @Override
        public void init(int op, String src, String dest, long max) {
            future.setBytesTotal(max);
        }

        @Override
        public boolean count(long count) {
            future.setBytesSoFar(count);
            return true;
        }

        @Override
        public void end() {

        }
    }
}
