package com.swrobotics.robot.logging;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private static final MechanismLigament2d shooterPivot = new MechanismLigament2d("Pivot", 0.7, 0, 3, new Color8Bit(Color.kGreen));
    static {
        shooter.append(shooterPivot);
    }

    public static void publish() {
        SmartDashboard.putData("Robot", view);
    }

    public static void updateIntake(double angleRot) {
        intakeUp.setAngle(90 + angleRot * 360);
    }

    // Rotation from horizontal
    public static void updateAmpArm(double armPivotRot) {
        double pivotRel = armPivotRot - referencePivotAngle;
        double intakeRel = -pivotRel * ampArmGearRatio;
        double intakeRot = referenceIntakeAngle + intakeRel;

        ampArmTop.setAngle(armPivotRot * 360);
        ampIntake.setAngle(intakeRot * 360 - 180);
    }

    public static void updateShooter(double pivotRot) {
        shooterPivot.setAngle(pivotRot * 360);
    }
}
