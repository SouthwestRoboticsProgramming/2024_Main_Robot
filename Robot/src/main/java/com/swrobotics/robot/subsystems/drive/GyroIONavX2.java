package com.swrobotics.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;

/* IO implementation for NavX2 */
public class GyroIONavX2 implements GyroIO {
    private final AHRS navx;

    public GyroIONavX2() {
        System.out.println("[Init] Creating GyroIONavX2");
        navx = new AHRS(Port.kMXP); // Connected over MXP on RoboRIO

        navx.calibrate(); // Calibrate the gyro (no need to call reset())
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.tempCelcius = navx.getTempC();
        inputs.connected = navx.isConnected();

        inputs.rollPositionRad = Units.degreesToRadians(navx.getRoll());
        inputs.pitchPositionRad = Units.degreesToRadians(navx.getPitch());
        inputs.yawPositionRad = Units.degreesToRadians(navx.getAngle());

        // The NavX2 does not have velocity functionality so those values are intentionally left blank
    }
}
