package com.swrobotics.robot.config;

/**
 * Class to define all CAN IDs in one place, so it is easy to verify all the IDs are set correctly
 */
public final class CANAllocation {
    public static final String CANIVORE_BUS = "geoff";

    // Drive, Turn, Encoder
    public static final SwerveIDs SWERVE_FL = new SwerveIDs(9, 5, 1);
    public static final SwerveIDs SWERVE_FR = new SwerveIDs(10, 6, 2);
    public static final SwerveIDs SWERVE_BL = new SwerveIDs(11, 7, 3);
    public static final SwerveIDs SWERVE_BR = new SwerveIDs(12, 8, 4);

    public static final class SwerveIDs {
        public final int drive, turn, encoder;

        public SwerveIDs(int drive, int turn, int encoder) {
            this.drive = drive;
            this.turn = turn;
            this.encoder = encoder;
        }
    }

    private CANAllocation() {
        throw new AssertionError();
    }
}
