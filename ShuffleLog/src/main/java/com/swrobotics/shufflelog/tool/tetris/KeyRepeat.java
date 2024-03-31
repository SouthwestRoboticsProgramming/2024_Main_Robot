package com.swrobotics.shufflelog.tool.tetris;

public final class KeyRepeat {
    private static final int repeatDelay = 175;
    private static final int repeatInterval = 60;

    private long downTime, repeatTime;
    private boolean down = false;
    private boolean justPressed = false;

    public void setDown(boolean down) {
        this.down = down;
        if (down) {
            downTime = System.currentTimeMillis();
            repeatTime = downTime + repeatDelay - repeatInterval;
            justPressed = true;
        }
    }

    public int getPressed() {
        if (justPressed) {
            justPressed = false;
            return 1;
        }

        if (!down)
            return 0;

        long time = System.currentTimeMillis() - downTime - repeatDelay;
        if (time < 0)
            return 0;

        int repeats = 0;
        while (System.currentTimeMillis() - repeatTime >= repeatInterval) {
            repeatTime += repeatInterval;
            repeats++;
        }
        return repeats;
    }
}
