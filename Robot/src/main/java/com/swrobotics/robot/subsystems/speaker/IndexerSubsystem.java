package com.swrobotics.robot.subsystems.speaker;

import com.swrobotics.lib.net.NTDouble;
import com.swrobotics.lib.net.NTString;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.config.NTData;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class IndexerSubsystem extends SubsystemBase {
    private final VictorSP topMotor = new VictorSP(IOAllocation.RIO.PWM_INDEXER_TOP_MOTOR);
    private final DigitalInput beamBreak = new DigitalInput(IOAllocation.RIO.DIO_INDEXER_BEAM_BREAK);
//    private final Debouncer beamBreakDebounce = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private final IntakeSubsystem intake;
    private boolean feedToShooter;

//    private boolean reverseTake, waitingForPiece;
//    private boolean hadPiece;
//    private boolean hasReindexed;

    public IndexerSubsystem(IntakeSubsystem intake) {
        this.intake = intake;

        feedToShooter = false;

        hadPiece = false;
    }

    public void setFeedToShooter(boolean feedToShooter) {
        this.feedToShooter = feedToShooter;
    }

    public boolean hasPiece() {
        // Beam break is true when not blocked
        return !beamBreak.get();
//        return !beamBreakDebounce.calculate(beamBreak.get());
    }

    // Idle state forward: feed in from intake
    //   When detect note, switch to reverse state
    // Reverse state: run in reverse
    //   When stopped detecting note, switch to index state
    // Index state: run forward
    //   When detect note, switch to stopped state
    // Stopped state: no run

    // When not intaking and not reverse intaking, switch to stopped state

    private enum State {
        IDLE,
        FEED_FORWARD,
        FEED_REVERSE,
        INDEX,
        HOLD
    }
    private State state = State.IDLE;
    private boolean hadPiece = false;
    private boolean reverseIntaking = false;
    private double stateStartTimestamp = Timer.getFPGATimestamp();

    private void switchState(State state) {
        stateStartTimestamp = Timer.getFPGATimestamp();
        this.state = state;
    }

    NTString stateDebug = new NTString("Debug/Indexer State", "");
    NTDouble stateTime = new NTDouble("Debug/Indexer State Time", 0);

    @Override
    public void periodic() {
        boolean hasPiece = hasPiece();
        NTData.INDEXER_HAS_PIECE.set(hasPiece);
        boolean intaking = intake.getState() == IntakeSubsystem.State.INTAKE;

        boolean anyRequest = intaking || reverseIntaking;
        if (anyRequest)
            switchState(state); // Prevent timeout while requesting

        // Time out if something goes wrong
        double time = Timer.getFPGATimestamp() - stateStartTimestamp;
        stateTime.set(time);
        stateDebug.set(state.toString());
        if (!anyRequest && time > 1) {
            switchState(State.IDLE);
        }

        double top = 0;
        if (feedToShooter) {
            top = NTData.INDEXER_TOP_FEED_SPEED.get();
        } else {
            switch (state) {
                case IDLE -> {
                    top = 0;
                    if (reverseIntaking)
                        switchState(State.FEED_REVERSE);
                    if (intaking)
                        switchState(State.FEED_FORWARD);
                }
                case FEED_FORWARD -> {
                    top = NTData.INDEXER_TOP_TAKE_SPEED.get();
                    if (hasPiece)
                        switchState(DriverStation.isAutonomous() ? State.HOLD : State.FEED_REVERSE);
                }
                case FEED_REVERSE -> {
                    top = -NTData.INDEXER_TOP_TAKE_SPEED.get();
                    if (!hasPiece && hadPiece) // Stopped seeing the note
                        switchState(State.INDEX);
                }
                case INDEX -> {
                    top = NTData.INDEXER_TOP_INDEX_SPEED.get();
                    if (hasPiece)
                        switchState(State.HOLD);
                }
                case HOLD -> {
                    top = 0;
                    if (!anyRequest)
                        switchState(State.IDLE);
                }
            }
        }

        topMotor.set(top);
        hadPiece = hasPiece;

//        boolean hasPiece = hasPiece();
//        NTData.INDEXER_HAS_PIECE.set(hasPiece);
//
//        boolean intaking = intake.getState() == IntakeSubsystem.State.INTAKE;
//
//        double sides = 0, top = 0;
//        if (feedToShooter) {
//            sides = NTData.INDEXER_SIDES_FEED_SPEED.get();
//            top = NTData.INDEXER_TOP_FEED_SPEED.get();
//        } else if (reverseTake && waitingForPiece) {
//            sides = -NTData.INDEXER_SIDES_FEED_SPEED.get();
//            top = -NTData.INDEXER_TOP_FEED_SPEED.get();
//
//            // Stop reversing once the piece has gone fully past the beam break
//            // Then the case below will run indexer forward to get it to normal position
//            if (!hasPiece && hadPiece)
//                waitingForPiece = false;
//        } else if ((intaking || reverseTake) && !hasPiece) {
//            sides = NTData.INDEXER_SIDES_TAKE_SPEED.get();
//            top = NTData.INDEXER_TOP_TAKE_SPEED.get();
//        }
//
//        sidesMotor.set(sides);
//        topMotor.set(top);
//
//        hadPiece = hasPiece;
    }

    public void beginReverse() {
        reverseIntaking = true;
//        reverseTake = true;
//        waitingForPiece = true;
    }

    public void endReverse() {
        reverseIntaking = false;
//        reverseTake = false;
    }
}
