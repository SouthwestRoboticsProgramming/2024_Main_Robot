package com.swrobotics.blockauto.util;

import java.util.concurrent.CompletableFuture;

public final class FutureUtils {
    // Assumes the future has already failed exceptionally
    public static Throwable getException(CompletableFuture<?> future) {
        return future.<Throwable>thenApply(v -> null).exceptionally(t -> t).join();
    }

    private FutureUtils() {
        throw new AssertionError();
    }
}
