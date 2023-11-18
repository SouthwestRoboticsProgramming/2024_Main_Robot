package com.swrobotics.robot;

import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTEntry;

/**
 * Class to store all tunable NetworkTables values in one place, to make it easier to hardcode the
 * defaults in case of data loss.
 *
 * <p>Temporary values should not be here, this class is only for persistent data.
 */
public final class NTData {
    public static final NTEntry<Double> FL_OFFSET = new NTDouble("Swerve/Modules/Front Left Offset", 0).setPersistent();
    public static final NTEntry<Double> FR_OFFSET = new NTDouble("Swerve/Modules/Front Right Offset", 0).setPersistent();
    public static final NTEntry<Double> BL_OFFSET = new NTDouble("Swerve/Modules/Back Left Offset", 0).setPersistent();
    public static final NTEntry<Double> BR_OFFSET = new NTDouble("Swerve/Modules/Back Right Offset", 0).setPersistent();
}
