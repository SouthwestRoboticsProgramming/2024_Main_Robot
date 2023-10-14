package com.swrobotics.robot.control.req;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public final class ControlRequest implements Comparable<ControlRequest> {
    public static final class Builder {
        private final Command command;
        private final Set<Subsystem> requirements;
        private int priority;
        private double lifeTime;

        public Builder(Command command) {
            this.command = command;
            requirements = new HashSet<>();
            lifeTime = Double.POSITIVE_INFINITY;
            priority = 0;
        }

        public Builder addRequirements(Subsystem... requirements) {
            this.requirements.addAll(Arrays.asList(requirements));
            return this;
        }

        public Builder setPriority(int priority) {
            this.priority = priority;
            return this;
        }

        public Builder setLifeTime(double lifeTimeSecs) {
            lifeTime = lifeTimeSecs;
            return this;
        }

        public ControlRequest build() {
            return new ControlRequest(this);
        }
    }

    private final double timestamp, expirationTime;
    private final int priority;
    private final Set<Subsystem> requirements;
    private final Command command;

    private ControlRequest(Builder builder) {
        timestamp = Timer.getFPGATimestamp();
        expirationTime = timestamp + builder.lifeTime;
        priority = builder.priority;
        requirements = builder.requirements;
        command = builder.command;
    }

    @Override
    public int compareTo(ControlRequest other) {
        // Sort by priority first, then timestamp
        int comparePriority = Integer.compare(priority, other.priority);
        if (comparePriority != 0)
            return comparePriority;
        return Double.compare(timestamp, other.timestamp);
    }
}
