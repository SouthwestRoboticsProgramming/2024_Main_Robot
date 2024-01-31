package com.swrobotics.robot.subsystems.speaker;

import com.revrobotics.CANSparkLowLevel;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.utils.SparkMaxWithSim;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class IntakeSubsystem extends SubsystemBase {
    private static final double motorToIntakeRatio = 2; // FIXME: Probably is not 2:1

    private final SparkMaxWithSim actuatorMotor = SparkMaxWithSim.create(
            IOAllocation.CAN.INTAKE_ACTUATOR_MOTOR,
            CANSparkLowLevel.MotorType.kBrushless,
            DCMotor.getNEO(1),
            motorToIntakeRatio,
            0.005);
    private final PWMTalonSRX spinMotor;

    private boolean active;
    private boolean hasCalibrated;
    private Debouncer actuatorStillDebounce;

    public IntakeSubsystem() {
        actuatorMotor.setPID(NTData.INTAKE_KP, NTData.INTAKE_KI, NTData.INTAKE_KD);
        actuatorMotor.setRotorToMechanismRatio(motorToIntakeRatio);
        actuatorMotor.setInverted(false); // FIXME

        spinMotor = new PWMTalonSRX(IOAllocation.RIO.PWM_INTAKE_MOTOR);
        actuatorStillDebounce = null;

        // The hard-stop the calibration relies on does not exist in simulation
        // The position is already correct anyway because the sim starts in a known state
        hasCalibrated = RobotBase.isSimulation();
    }

    public void set(boolean active) {
        this.active = active;
        if (!hasCalibrated)
            return;

        actuatorMotor.setPosition(active ? NTData.INTAKE_RANGE.get() / 360 : 0);
        spinMotor.set(active ? NTData.INTAKE_SPEED.get() : 0);
    }

    public boolean isActive() {
        return active;
    }

    @Override
    public void periodic() {
        if (NTData.INTAKE_RECALIBRATE.get()) {
            hasCalibrated = false;
            actuatorStillDebounce = null;
        }
        if (hasCalibrated || DriverStation.isDisabled())
            return;

        // Defer debouncer initialization until now so the first edge still applies
        if (actuatorStillDebounce == null) {
            // Defaults to false, which gives the motor a little time to start
            // moving before we stop
            actuatorStillDebounce = new Debouncer(NTData.INTAKE_CALIBRATE_DEBOUNCE.get(), Debouncer.DebounceType.kBoth);
        }

        System.out.println("Calibration: encoder velocity = " + actuatorMotor.getEncoderVelocity());
        boolean isStill = Math.abs(actuatorMotor.getEncoderVelocity()) < NTData.INTAKE_CALIBRATE_STALL_THRESHOLD.get();
        if (actuatorStillDebounce.calculate(isStill)) {
            hasCalibrated = true;

            // Fully retracted now, set position
            // Slightly negative so the motor doesn't stall on the hard-stop
            // when retracting
            actuatorMotor.setEncoderPosition(-5);
            set(active);
        } else {
            actuatorMotor.setVoltage(NTData.INTAKE_CALIBRATE_VOLTS.get());
        }
    }

    @Override
    public void simulationPeriodic() {
        actuatorMotor.updateSim(12);
        SimView.updateIntake(actuatorMotor.getEncoderPosition());
    }
}
