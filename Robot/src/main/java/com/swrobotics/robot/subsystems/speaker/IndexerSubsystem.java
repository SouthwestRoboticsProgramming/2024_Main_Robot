package com.swrobotics.robot.subsystems.speaker;

import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class IndexerSubsystem extends SubsystemBase {
    public enum State {
        TAKE_FROM_INTAKE,
        FEED_TO_SHOOTER
    }

    private final PWMTalonSRX sidesMotor = new PWMTalonSRX(IOAllocation.RIO.PWM_INDEXER_SIDES_MOTOR);
    private final PWMTalonSRX topMotor = new PWMTalonSRX(IOAllocation.RIO.PWM_INDEXER_TOP_MOTOR);
    private final DigitalInput beamBreak = new DigitalInput(IOAllocation.RIO.DIO_INDEXER_BEAM_BREAK);
    private final Debouncer beamBreakDebounce = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private final IntakeSubsystem intake;
    private State state;

    public IndexerSubsystem(IntakeSubsystem intake) {
        this.intake = intake;
        state = State.TAKE_FROM_INTAKE;
    }

    public void setState(State state) {
        this.state = state;
    }

    public boolean hasPiece() {
        // Beam break is true when not blocked
        return !beamBreakDebounce.calculate(beamBreak.get());
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled())
            return;

        double sides = 0, top = 0;
        if (state == State.TAKE_FROM_INTAKE) {
            if (intake.isActive() && !hasPiece()) {
                sides = NTData.INDEXER_SIDES_TAKE_SPEED.get();
                top = NTData.INDEXER_TOP_TAKE_SPEED.get();
            }
        } else {
            sides = NTData.INDEXER_SIDES_FEED_SPEED.get();
            top = NTData.INDEXER_TOP_FEED_SPEED.get();
        }

        sidesMotor.set(sides);
        topMotor.set(top);
    }
}
