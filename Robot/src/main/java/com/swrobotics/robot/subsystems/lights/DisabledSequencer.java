package com.swrobotics.robot.subsystems.lights;

import com.swrobotics.robot.subsystems.lights.LightsSubsystem.Stripe;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.wpilibj.util.Color.*;

public final class DisabledSequencer {
    private static final Stripe[] TEST_1 = create(kRed, kGreen, kBlue);
    private static final Stripe[] TEST_2 = create(kYellow, kCyan, kMagenta);

    private static final Stripe[][] PATTERNS = {
            TEST_1, TEST_2
    };

    private static final int REPEAT = 2;
    private static final float SCROLL = 2f;
    private static final double PRESENT_TIME = 10;
    private static final double TRANSITION_TIME = 0.5;

    private static Color rgb(int r, int g, int b) {
        return new Color(r / 255.0, g / 255.0, b / 255.0);
    }

    private static Stripe[] create(Color... colors) {
        Stripe[] stripes = new Stripe[colors.length];
        for (int i = 0; i < colors.length; i++) {
            stripes[i] = new Stripe(colors[i], 1);
        }
        return stripes;
    }

    private Stripe[] current;
    private int currentIdx;
    private double currentStartTimestamp;

    public DisabledSequencer() {
        currentIdx = Integer.MAX_VALUE;
        selectNext();
    }

    private void selectNext() {
        int random = (int) (Math.random() * (PATTERNS.length - 1));
        if (random >= currentIdx)
            random += 1;
        currentIdx = random;
        Stripe[] once = PATTERNS[random];
        current = new Stripe[once.length * REPEAT];
        for (int i = 0; i < REPEAT; i++) {
            System.arraycopy(once, 0, current, i * once.length, once.length);
        }
        currentStartTimestamp = Timer.getFPGATimestamp();
    }

    private Stripe[] darken(Stripe[] pattern, double bright) {
        Stripe[] newPattern = new Stripe[pattern.length];
        for (int i = 0; i < pattern.length; i++) {
            Color col = pattern[i].color();
            Color newCol = new Color(
                    col.red * bright,
                    col.green * bright,
                    col.blue * bright
            );
            newPattern[i] = new Stripe(newCol, pattern[i].weight());
        }
        return newPattern;
    }

    // Don't have to care about running quickly here, since it's in disabled
    // We can have whatever fancy effects we want
    public void apply(LightsSubsystem lights) {
        double time = Timer.getFPGATimestamp();
        double elapsed = time - currentStartTimestamp;

        if (elapsed > PRESENT_TIME + TRANSITION_TIME * 2) {
            selectNext();
            elapsed = 0;
        }

        Stripe[] toSet = current;
        if (elapsed < TRANSITION_TIME)
            toSet = darken(current, elapsed / TRANSITION_TIME);
        else if (elapsed > PRESENT_TIME + TRANSITION_TIME)
            toSet = darken(current, 1 - (elapsed - PRESENT_TIME - TRANSITION_TIME) / TRANSITION_TIME);

        lights.applyStripes(SCROLL, toSet);
    }

    public void reset() {
        selectNext();
    }
}
