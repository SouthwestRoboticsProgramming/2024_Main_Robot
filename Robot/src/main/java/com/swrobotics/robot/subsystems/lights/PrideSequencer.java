package com.swrobotics.robot.subsystems.lights;

import com.swrobotics.robot.subsystems.lights.LightsSubsystem.Stripe;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
import java.util.List;

public final class PrideSequencer {
    private static final record Flag(Stripe[] stripes, int weight) {}

    private static final Flag[] FLAGS = {
            // 4x pride so it shows up more often
            createFlag("ff1e26,fe941e,ffff00,06bd00,001a98,760088", 4), // Pride

            // 2x the ones people on the team are
            createFlag("000000,9e9e9e,ffffff,5e1984", 2), // Asexual
            createFlag("55cdfd,f6aa87,ffffff,f6aa87,55cdfd", 2), // Transgender
            createFlag("d70071*2,9c4e97,0035aa*2", 2), // Bisexual
            createFlag("fcf431,ffffff,9d59d2,000000", 2), // Nonbinary
            createFlag("3da542,a7d379,ffffff,a9a9a9,000000", 2), // Aromantic
            createFlag("078d70,26ceaa,98e8c1,ffffff,7bade2,5049cc,3d1a78", 2), // Gay

            createFlag("000000,bcc6c7,ffffff,a6fa6e,ffffff,bcc6c7,000000"), // Agender
            createFlag("fa4288,ffffff,71197f,000000,071195"), // Genderfluid
            createFlag("d42c00,fd9855,ffffff,d161a2,a20161"), // Lesbian
            createFlag("b57edc,ffffff,4a8122"), // Bigender
            createFlag("ffffff*4,7e287f,a3a3a3*4"), // Demisexual
            createFlag("fe218b,fed700,21b0fe"), // Pansexual
    };

    private static Flag createFlag(String colors) { return createFlag(colors, 1); }
    private static Flag createFlag(String colors, int patternWeight) {
        String[] hexCodes = colors.split(",");
        List<Stripe> stripes = new ArrayList<>();
        for (String hexCode : hexCodes) {
            int r = Integer.parseInt(hexCode.substring(0, 2), 16);
            int g = Integer.parseInt(hexCode.substring(2, 4), 16);
            int b = Integer.parseInt(hexCode.substring(4, 6), 16);

            float weight = 1;
            int weightIdx = hexCode.indexOf('*');
            if (weightIdx >= 0) {
                weight = Float.parseFloat(hexCode.substring(weightIdx + 1));
            }

            stripes.add(new Stripe(new Color(r / 255.0, g / 255.0, b / 255.0), weight));
        }

        // If first and last stripes are the same color, remove one so the
        // pattern looks nicer when it repeats
        if (stripes.size() > 1) {
            Stripe first = stripes.get(0);
            Stripe last = stripes.get(stripes.size() - 1);

            if (first.color().equals(last.color())) {
                if (first.weight() > last.weight()) {
                    stripes.remove(stripes.size() - 1);
                } else {
                    stripes.remove(0);
                }
            }
        }

        return new Flag(stripes.toArray(new Stripe[0]), patternWeight);
    }

    private static final int REPEAT = 1;
    private static final float SCROLL = 4f;
    private static final double PRESENT_TIME = 10;
    private static final double TRANSITION_TIME = 0.5;

    private Stripe[] current;
    private int currentIdx;
    private double currentStartTimestamp;

    public PrideSequencer() {
        currentIdx = Integer.MAX_VALUE;
        selectNext();
    }

    private void selectNext() {
        int totalWeight = 0;
        for (int i = 0; i < FLAGS.length; i++) {
            if (i != currentIdx) {
                totalWeight += FLAGS[i].weight;
            }
        }

        int randomWeight = (int) (Math.random() * totalWeight);
        Stripe[] flagStripes = FLAGS[FLAGS.length - (currentIdx == FLAGS.length - 1 ? 2 : 1)].stripes;
        int acc = 0;
        for (int i = 0; i < FLAGS.length; i++) {
            if (i != currentIdx) {
                acc += FLAGS[i].weight;
                if (acc > randomWeight) {
                    flagStripes = FLAGS[i].stripes;
                    currentIdx = i;
                    break;
                }
            }
        }

        current = new Stripe[flagStripes.length * REPEAT];
        for (int i = 0; i < REPEAT; i++) {
            System.arraycopy(flagStripes, 0, current, i * flagStripes.length, flagStripes.length);
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
