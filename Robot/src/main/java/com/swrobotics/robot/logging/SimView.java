package com.swrobotics.robot.logging;

import com.swrobotics.mathlib.MathUtil;
import com.swrobotics.robot.subsystems.speaker.IntakeSubsystem;
import com.swrobotics.robot.subsystems.speaker.IntakeSubsystem.State;
import com.swrobotics.robot.subsystems.speaker.aim.AimCalculator;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

// FIXME: All dimensions in here are wrong
public final class SimView {
    // Drawn as side view such that robot facing to the right

    private static final Mechanism2d view = new Mechanism2d(2, 2);
    private static final double originX = 1;
    private static final double originY = 0.5;

    private static final MechanismRoot2d intake = view.getRoot("Intake", originX - 0.5, originY);
    private static final MechanismLigament2d intakeUp = new MechanismLigament2d("Up", 0.25, 90, 3, new Color8Bit(Color.kDarkGoldenrod));
    static {
        MechanismLigament2d intakeRight = new MechanismLigament2d("Right", 0.25, 90, 2, new Color8Bit(Color.kGoldenrod));
        intakeUp.append(intakeRight);
        intake.append(intakeUp);
    }

    private static final double referencePivotAngle = -45 / 360.0;
    private static final double referenceIntakeAngle = 135 / 360.0;
    private static final double ampArmGearRatio = 3.5 / 1.0;
    private static final MechanismRoot2d ampArm = view.getRoot("Amp Arm", originX, originY + 0.4);
    private static final MechanismLigament2d ampArmBottom = new MechanismLigament2d("Bottom", 0.4, -90, 4, new Color8Bit(Color.kDarkRed));
    private static final MechanismLigament2d ampArmTop = new MechanismLigament2d("Top", 0.3, 0, 3, new Color8Bit(Color.kRed));
    private static final MechanismLigament2d ampIntake = new MechanismLigament2d("Intake", 0.15, 0, 2, new Color8Bit(Color.kPink));
    static {
        ampArmTop.append(ampIntake);
        ampArm.append(ampArmBottom);
        ampArm.append(ampArmTop);
    }

    private static final MechanismRoot2d shooter = view.getRoot("Shooter", originX - 0.5, originY);
    private static final double maxShooterLength = 0.7;
    private static final MechanismLigament2d shooterPivot = new MechanismLigament2d("Pivot", maxShooterLength, 0, 3, new Color8Bit(Color.kGreen));
    static {
        shooter.append(shooterPivot);
    }

    public static final ShooterTrajectoryView lobTrajectory = new ShooterTrajectoryView(
            view.getRoot("Lob Trajectory", originX, originY + 0.4),
            new Color8Bit(Color.kYellow)
    );

    public static void publish() {
        SmartDashboard.putData("Robot", view);
    }

    public static void updateIntake(double angleRot) {
        intakeUp.setAngle(90 + angleRot * 360);
    }

    public static void updateIntake(IntakeSubsystem.State state) {
        if (state == State.OFF) {
            intakeUp.setAngle(90);
        } else {
            intakeUp.setAngle(180);
        }
    }

    // Rotation from horizontal
    public static void updateAmpArm(double armPivotRot) {
        double pivotRel = armPivotRot - referencePivotAngle;
        double intakeRel = -pivotRel * ampArmGearRatio;
        double intakeRot = referenceIntakeAngle + intakeRel;

        ampArmTop.setAngle(armPivotRot * 360);
        ampIntake.setAngle(intakeRot * 360 - 180);
    }

    public static void updateShooter(AimCalculator.Aim aim) {
        shooterPivot.setAngle(Math.toDegrees(aim.pivotAngle()));
        shooterPivot.setLength(aim.flywheelVelocity() / 100 * maxShooterLength);
    }

    public static void setShooting(boolean shooting) {
        if (shooting) {
            shooterPivot.setColor(new Color8Bit(Color.kBlueViolet));
            view.setBackgroundColor(new Color8Bit(Color.kYellow));
        } else {
            shooterPivot.setColor(new Color8Bit(Color.kGreen));
            view.setBackgroundColor(new Color8Bit(Color.kDarkGray));
        }
    }

    // Do not use on real robot ever! This is very inefficient and only for debugging sim
    public static final class ShooterTrajectoryView {
        private static final double maxTime = 2;
        private static final int segments = 16;

        private final MechanismLigament2d[] ligaments;

        public ShooterTrajectoryView(MechanismRoot2d source, Color8Bit color) {
            ligaments = new MechanismLigament2d[segments];

            // Make a big line of ligaments
            MechanismLigament2d prev = new MechanismLigament2d("Segment 0", 0, 0, 2, color);
            ligaments[0] = prev;
            source.append(prev);
            for (int i = 1; i < segments; i++) {
                MechanismLigament2d ligament = new MechanismLigament2d("Segment " + i, 0, 0, 2, color);
                prev.append(ligament);
                ligaments[i] = ligament;
                prev = ligament;
            }
        }

        public void clear() {
            for (MechanismLigament2d ligament : ligaments) {
                ligament.setLength(0);
            }
        }

        public void update(double initialVelocityMPS, double pivotAngleRad) {
            double vx = initialVelocityMPS * Math.cos(pivotAngleRad);
            double vy = initialVelocityMPS * Math.sin(pivotAngleRad);

            // Plot free-fall trajectory
            double px = 0, py = 0;
            double pa = 0;
            for (int i = 0; i < segments; i++) {
                double time = maxTime * (i + 1) / segments;

                double x = vx * time;
                double y = vy * time - 0.5 * MathUtil.G_ACCEL * time * time;
                double dx = x - px;
                double dy = y - py;
                px = x;
                py = y;

                double angle = Math.toDegrees(Math.atan2(dy, dx));
                MechanismLigament2d ligament = ligaments[i];
                ligament.setLength(Math.hypot(dx, dy));
                ligament.setAngle(angle - pa);
                pa = angle;
            }
        }
    }
}
