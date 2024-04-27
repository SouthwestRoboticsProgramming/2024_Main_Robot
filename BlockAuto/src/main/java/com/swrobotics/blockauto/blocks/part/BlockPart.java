package com.swrobotics.blockauto.blocks.part;

public interface BlockPart {
    // Whether the part adds onto the end of the line or starts a new line
    default boolean isInline() { return true; }

    default boolean isFramed() { return false; }
}
