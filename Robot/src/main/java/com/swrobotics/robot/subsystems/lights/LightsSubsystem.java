package com.swrobotics.robot.subsystems.lights;

import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.RobotContainer;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.subsystems.climber.ClimberArm;
import com.swrobotics.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public final class LightsSubsystem extends SubsystemBase {
    private static final int LED_COUNT = 22;

    private final RobotContainer robot;
    private final AddressableLED leds;
    private final AddressableLEDBuffer data;

    private final Debouncer batteryLowDebounce;
    private final PrideSequencer prideSequencer;

    private double shooterReadyStartTimestamp = Double.NaN;

    public LightsSubsystem(RobotContainer robot) {
        this.robot = robot;
        leds = new AddressableLED(IOAllocation.RIO.PWM_LEDS);
        leds.setLength(LED_COUNT);

        data = new AddressableLEDBuffer(LED_COUNT);
        leds.setData(data);

        leds.start();

        batteryLowDebounce = new Debouncer(10);
        prideSequencer = new PrideSequencer();
    }

    private void showLowBattery() {
        applySolid(Timer.getFPGATimestamp() % 0.5 > 0.25 ? Color.kRed : Color.kBlack);
    }

    private void showShooterStatus() {
        if (robot.shooter.isReadyToShoot()) {
            double timestamp = Timer.getFPGATimestamp();
            if (Double.isNaN(shooterReadyStartTimestamp))
                shooterReadyStartTimestamp = timestamp;
            double elapsed = timestamp - shooterReadyStartTimestamp;

            elapsed *= 4; // 4 blinks/sec

            // 2 blinks, on 50% of the time, solid after 3rd blink
            if ((elapsed <= 2 && (elapsed % 1) < 0.5) || elapsed > 2) {
                applySolid(Color.kLime);
            } else {
                applySolid(Color.kBlack);
            }
        } else {
            shooterReadyStartTimestamp = Double.NaN;
            double pct = robot.shooter.getFlywheelPercentOfTarget();
            if (pct < 1) {
                applyStripes(
                        0,
                        new Stripe(Color.kWhite, (float) pct),
                        new Stripe(Color.kPurple, 1 - (float) pct)
                );
            } else {
                float reversePct = (float) Math.min(pct - 1, 1);
                applyStripes(
                        0,
                        new Stripe(Color.kWhite, 1 - reversePct),
                        new Stripe(Color.kOrange, reversePct)
                );
            }
        }
    }

    private void showAutoDriving() {
        // Rainbow
        applyStripes(5f,
                new Stripe(Color.kRed, 1),
                new Stripe(Color.kOrange, 1),
                new Stripe(Color.kYellow, 1),
                new Stripe(Color.kGreen, 1),
                new Stripe(Color.kBlue, 1),
                new Stripe(Color.kPurple, 1));
    }

    private void showIdle() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        Color color = alliance
                .map(value -> value == DriverStation.Alliance.Blue ? Color.kBlue : Color.kRed)
                .orElse(Color.kPurple);

        applySolid(color);
    }

    @Override
    public void periodic() {
        // Special indicator to swap battery
        boolean batteryLow = RobotController.getBatteryVoltage() < 10;

        boolean resetShooterBlink = true;
        if (batteryLowDebounce.calculate(batteryLow)) {
            showLowBattery();
        } else if (DriverStation.isDisabled()) {
            prideSequencer.apply(this);
        } else if (robot.shooter.isPreparing()) {
            resetShooterBlink = false;
            showShooterStatus();
        } else if (robot.drive.getLastSelectedPriority() == SwerveDrive.AUTO_PRIORITY) {
            showAutoDriving();
        } else {
            showIdle();
        }

        if (resetShooterBlink)
            shooterReadyStartTimestamp = Double.NaN;
    }

    private void applySolid(Color color) {
        for (int i = 0; i < LED_COUNT; i++) {
            data.setLED(i, color);
        }
        leds.setData(data);
    }

    public static final record Stripe(Color color, float weight) {}

    private Color interpolate(Color a, Color b, float percent) {
        return new Color(
                MathUtil.lerp(a.red, b.red, percent),
                MathUtil.lerp(a.green, b.green, percent),
                MathUtil.lerp(a.blue, b.blue, percent)
        );
    }

    private static final record StripeBoundary(float position, Stripe toRight) {}

    // Scroll speed is seconds per full pass through the pattern
    // TODO: optimize this
    public void applyStripes(float scrollSpeed, Stripe... stripes) {
        float totalWeight = 0;
        for (Stripe stripe : stripes)
            totalWeight += stripe.weight;

        float scroll = scrollSpeed == 0 ? 0 : (float) Timer.getFPGATimestamp() / scrollSpeed;

        // Pattern is sampled at pixel's left edge
        List<StripeBoundary> boundaries = new ArrayList<>();
        float weightSoFar = 0;
        for (Stripe stripe : stripes) {
            float leftEdgePos = (float) MathUtil.floorMod(weightSoFar / totalWeight + scroll, 1) * LED_COUNT;
            boundaries.add(new StripeBoundary(leftEdgePos, stripe));
            weightSoFar += stripe.weight;
        }
        boundaries.sort(Comparator.comparingDouble(StripeBoundary::position));

        for (int pixel = 0; pixel < LED_COUNT; pixel++) {
            StripeBoundary before = null, after = null;
            for (StripeBoundary boundary : boundaries) {
                if (boundary.position > pixel) {
                    after = boundary;
                    break;
                }
                before = boundary;
            }

            if (before == null)
                before = boundaries.get(boundaries.size() - 1);
            if (after == null) {
                data.setLED(pixel, before.toRight.color);
                continue;
            }

            if (pixel == (int) after.position) {
                float percent = 1 - (after.position % 1);
                data.setLED(pixel, interpolate(before.toRight.color, after.toRight.color, percent));
                continue;
            }

            data.setLED(pixel, before.toRight.color);
        }

        leds.setData(data);
    }

    public void disabledInit() {
        prideSequencer.reset();
    }
}
