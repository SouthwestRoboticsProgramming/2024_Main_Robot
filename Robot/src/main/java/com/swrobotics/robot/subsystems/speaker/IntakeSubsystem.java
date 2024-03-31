package com.swrobotics.robot.subsystems.speaker;

import com.revrobotics.CANSparkLowLevel;
import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import com.swrobotics.robot.logging.SimView;
import com.swrobotics.robot.utils.SparkMaxWithSim;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class IntakeSubsystem extends SubsystemBase {
    public enum State {
        INTAKE,
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

    private final PowerDistribution pdp;
    private State state;
    private boolean hasCalibrated;
    private Debouncer actuatorStillDebounce;

    private boolean reverse;

    public IntakeSubsystem(PowerDistribution pdp) {
        this.pdp = pdp;

        actuatorMotor.setPID(NTData.INTAKE_KP, NTData.INTAKE_KD);
        actuatorMotor.setRotorToMechanismRatio(motorToIntakeRatio);
        actuatorMotor.setInverted(false); // FIXME

        spinMotor = new PWMSparkMax(IOAllocation.RIO.PWM_INTAKE_MOTOR);
        spinMotor.setInverted(true);
        actuatorStillDebounce = null;

        // The hard-stop the calibration relies on does not exist in simulation
        // The position is already correct anyway because the sim starts in a known state
        hasCalibrated = RobotBase.isSimulation();
        state = State.OFF;

        reverse = false;
    }

    public void set(State state) {
        this.state = state;
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
        if (DriverStation.isDisabled())
            return;

        if (!hasCalibrated) {
            hasCalibrated = true;
            actuatorMotor.setEncoderPosition(-NTData.INTAKE_CALIBRATE_SETPOINT.get() / 360.0);
        }

        boolean extend = state != State.OFF;
        double speed = switch (state) {
            case INTAKE -> NTData.INTAKE_SPEED.get();
            case OFF -> 0;
        };
        if (reverse)
            speed = -1;

        // Manual voltage compensation
        // FIXME: This may be why the intake sometimes "wiggles"
        //  in the sim supplyVolts is sometimes randomly 0 when using PDP
//        double supplyVolts = pdp.getVoltage();
        double supplyVolts = RobotController.getBatteryVoltage(); // Use RIO power input instead (should be same voltage)
        if (supplyVolts != 0) {
            double comp = 12.0 / supplyVolts;
            speed *= comp;
        }

        actuatorMotor.setPosition(extend ? NTData.INTAKE_RANGE.get() / 360 : 0);
        double spinOut = MathUtil.clamp(speed, -1, 1);
        spinMotor.set(spinOut);
        out.set(spinOut);
//        System.out.println("Intake: " + speed + " -> " + spinOut + " (Supply " + supplyVolts + "V)");
    }

    NTDouble out = new NTDouble("Intake/Debug out", 1234);

    @Override
    public void simulationPeriodic() {
        // actuatorMotor.updateSim(12);
        // SimView.updateIntake(actuatorMotor.getEncoderPosition());
        SimView.updateIntake(state);
    }

    public void setReverse(boolean reverse) {
        this.reverse = reverse;
    }
}
