package com.swrobotics.shufflelog.tool.tetris;

public enum Rotation {
    NONE(0),
    CW_90(1),
    R_180(2),
    CCW_90(3);

    static {
        NONE.next = CW_90;
        CW_90.next = R_180;
        R_180.next = CCW_90;
        CCW_90.next = NONE;
    }

    public final int kickIdx;
    private Rotation next;

    Rotation(int kickIdx) {
        this.kickIdx = kickIdx;
    }

    public Rotation next() {
        return next;
    }
}
