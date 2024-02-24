package com.swrobotics.robot.subsystems.speaker;

import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class IndexerSubsystem extends SubsystemBase {
    private final PWMTalonSRX sidesMotor = new PWMTalonSRX(IOAllocation.RIO.PWM_INDEXER_SIDES_MOTOR);
    private final PWMVictorSPX topMotor = new PWMVictorSPX(IOAllocation.RIO.PWM_INDEXER_TOP_MOTOR);
    private final DigitalInput beamBreak = new DigitalInput(IOAllocation.RIO.DIO_INDEXER_BEAM_BREAK);
    private final Debouncer beamBreakDebounce = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private final IntakeSubsystem intake;
    private boolean feedToShooter;
    private boolean reverse;

    public IndexerSubsystem(IntakeSubsystem intake) {
        this.intake = intake;

        sidesMotor.setInverted(true);

        feedToShooter = false;
    }

    public void setFeedToShooter(boolean feedToShooter) {
        this.feedToShooter = feedToShooter;
    }

    public boolean hasPiece() {
        // Beam break is true when not blocked
        return !beamBreakDebounce.calculate(beamBreak.get());
    }

    @Override
    public void periodic() {
        NTData.INDEXER_HAS_PIECE.set(hasPiece());

        double sides = 0, top = 0;
        if (feedToShooter) {
            sides = NTData.INDEXER_SIDES_FEED_SPEED.get();
            top = NTData.INDEXER_TOP_FEED_SPEED.get();
        } else {
            if (intake.getState() == IntakeSubsystem.State.INTAKE && !hasPiece()) {
                sides = NTData.INDEXER_SIDES_TAKE_SPEED.get();
                top = NTData.INDEXER_TOP_TAKE_SPEED.get();
            }
        }

        if (reverse) {
            sides = -NTData.INDEXER_SIDES_FEED_SPEED.get();
            top = -NTData.INDEXER_TOP_FEED_SPEED.get();
        }

        sidesMotor.set(sides);
        topMotor.set(top);
    }

    public void setReverse(boolean reverse) {
        this.reverse = reverse;
    }
}
