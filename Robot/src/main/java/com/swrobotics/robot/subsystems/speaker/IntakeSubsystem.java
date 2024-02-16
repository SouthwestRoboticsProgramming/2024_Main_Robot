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
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class IntakeSubsystem extends SubsystemBase {
    public enum State {
        INTAKE,
        EJECT,
        OFF
    }

    private static final double motorToIntakeRatio = 25 * (36 / 16.0);

    private final SparkMaxWithSim actuatorMotor = SparkMaxWithSim.create(
            IOAllocation.CAN.INTAKE_ACTUATOR_MOTOR,
            CANSparkLowLevel.MotorType.kBrushless,
            DCMotor.getNEO(1),
            motorToIntakeRatio,
            0.005);
    private final PWMSparkMax spinMotor;

    private State state;
    private boolean hasCalibrated;
    private Debouncer actuatorStillDebounce;

    public IntakeSubsystem() {
        actuatorMotor.setPID(NTData.INTAKE_KP, NTData.INTAKE_KI, NTData.INTAKE_KD);
        actuatorMotor.setRotorToMechanismRatio(motorToIntakeRatio);
        actuatorMotor.setInverted(false); // FIXME

        spinMotor = new PWMSparkMax(IOAllocation.RIO.PWM_INTAKE_MOTOR);
        spinMotor.setInverted(true);
        actuatorStillDebounce = null;

        // The hard-stop the calibration relies on does not exist in simulation
        // The position is already correct anyway because the sim starts in a known state
        hasCalibrated = RobotBase.isSimulation();
        state = State.OFF;
    }

    public void set(State state) {
        this.state = state;
        if (!hasCalibrated)
            return;

        boolean extend = state != State.OFF;
        double speed = switch (state) {
            case INTAKE -> NTData.INTAKE_SPEED.get();
            case EJECT -> -NTData.INTAKE_EJECT_SPEED.get();
            case OFF -> 0;
        };

        Logger.recordOutput("intake setpoint", extend ? NTData.INTAKE_RANGE.get() / 360 : 0);
        Logger.recordOutput("intake current", actuatorMotor.getEncoderPosition());

        actuatorMotor.setPosition(extend ? NTData.INTAKE_RANGE.get() / 360 : 0);
        spinMotor.set(speed);
    }

    public State getState() {
        return state;
    }

    @Override
    public void periodic() {
        if (NTData.INTAKE_RECALIBRATE.get()) {
            NTData.INTAKE_RECALIBRATE.set(false);
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

        boolean isStill = Math.abs(actuatorMotor.getEncoderVelocity()) < NTData.INTAKE_CALIBRATE_STALL_THRESHOLD.get();
        if (actuatorStillDebounce.calculate(isStill)) {
            hasCalibrated = true;

            // Fully retracted now, set position
            actuatorMotor.setEncoderPosition(-NTData.INTAKE_CALIBRATE_SETPOINT.get() / 360.0);
            set(state);
            NTData.INTAKE_CALIBRATING.set(false);
        } else {
            actuatorMotor.setVoltage(-NTData.INTAKE_CALIBRATE_VOLTS.get());
            NTData.INTAKE_CALIBRATING.set(true);
        }
    }

    @Override
    public void simulationPeriodic() {
        actuatorMotor.updateSim(12);
        SimView.updateIntake(actuatorMotor.getEncoderPosition());
    }
}
