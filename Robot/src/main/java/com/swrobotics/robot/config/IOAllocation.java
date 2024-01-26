package com.swrobotics.robot.config;

import edu.wpi.first.wpilibj.RobotBase;

public final class IOAllocation {
    public static final class CanId {
        public static int uniqueifyForSim(int id, String bus) {
            // CTRE sim doesn't support CANivore, so ids need to be globally unique
            if (RobotBase.isSimulation() && bus.equals(CAN.GERALD))
                return id + 32;

            return id;
        }

        private final int id;
        private final String bus;

        public CanId(int id, String bus) {
            this.id = id;
            this.bus = bus;
        }

        public int id() {
            return uniqueifyForSim(id, bus);
        }

        public String bus() {
            return bus;
        }
    }

    public static final class CAN {
        private static final String RIO = "";
        public static final String GERALD = "Gerald";

        // Gerald
        // Drive, Turn, Encoder
        public static final SwerveIDs SWERVE_FL = new SwerveIDs(9, 5, 1);  // falcon + kraken
        public static final SwerveIDs SWERVE_FR = new SwerveIDs(10, 6, 2); // falcon + kraken
        public static final SwerveIDs SWERVE_BL = new SwerveIDs(11, 7, 3); // falcon + kraken
        public static final SwerveIDs SWERVE_BR = new SwerveIDs(12, 8, 4); // falcon + kraken

        public static final CanId CLIMBER_L_MOTOR = new CanId(14, GERALD); // falcon
        public static final CanId CLIMBER_R_MOTOR = new CanId(15, GERALD); // falcon

        public static final CanId AMP_ARM_MOTOR = new CanId(16, GERALD); // falcon

        // RIO
        public static final CanId SHOOTER_MOTOR_1 = new CanId(1, RIO); // falcon -> kraken
        public static final CanId SHOOTER_MOTOR_2 = new CanId(2, RIO); // falcon -> kraken
        public static final CanId SHOOTER_PIVOT_MOTOR = new CanId(3, RIO); // neo
        public static final CanId SHOOTER_PIVOT_CANCODER = new CanId(4, RIO);

        public static final CanId INTAKE_ACTUATOR_MOTOR = new CanId(5, RIO); // neo
        public static final CanId INTAKE_ACTUATOR_CANCODER = new CanId(6, RIO);

        public static final CanId AMP_ARM_CANCODER = new CanId(7, RIO);

        public static final CanId PDP = new CanId(62, RIO);
    }

    public static final class RIO {
        public static final int PWM_INTAKE_MOTOR = 0; // NEO
        public static final int PWM_AMP_INTAKE_MOTOR = 1; // small

        // Should have brake mode saved into SRX flash
        // Use button so that red light is ON (indicates brake mode is on)
        public static final int PWM_INDEXER_SIDES_MOTOR = 2; // something
        public static final int PWM_INDEXER_TOP_MOTOR = 3; // something 2

        public static final int PWM_LEDS = 4;

        public static final int DIO_INDEXER_BEAM_BREAK = 0;
    }

    public static final class SwerveIDs {
        public final int drive, turn, encoder;

        public SwerveIDs(int drive, int turn, int encoder) {
            this.drive = CanId.uniqueifyForSim(drive, CAN.GERALD);
            this.turn = CanId.uniqueifyForSim(turn, CAN.GERALD);
            this.encoder = CanId.uniqueifyForSim(encoder, CAN.GERALD);
        }
    }
}
