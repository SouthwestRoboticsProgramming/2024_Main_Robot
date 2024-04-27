package com.swrobotics.blockauto.blocks.part;

public interface ParameterPart extends BlockPart {
    Object getDefaultValue();

    // Returns the new value
    Object edit(Object value);

    @Override
    default boolean isFramed() {
        return true;
    }

    default Object copyValue(Object value) {
        return value;
    }
}
