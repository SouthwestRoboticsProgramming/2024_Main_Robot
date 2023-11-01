package com.swrobotics.robot.subsystems.drive;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class KinematicLimits {
    public double maxTranslationalVelocity = DriveConstants.MAX_SPEED; // ms^-1
    public double maxTranslationalAcceleration = Double.MAX_VALUE; // ms^-2
    public double maxAngularVelocity = DriveConstants.MAX_ANGULAR_SPEED; // rad/s
    public double maxAngularAcceleration = Double.MAX_VALUE; // rad/s^2

    private Translation2d netAcceleration = new Translation2d();

    public ChassisSpeeds limitSpeeds(ChassisSpeeds requestedSpeeds, ChassisSpeeds currentSpeeds) {
        // TODO: Remove
        double robotMass = Units.lbsToKilograms(125.0);
        Translation3d cg = new Translation3d(0.0, 1.0, Units.inchesToMeters(12));

        // Break ChassisSpeeds into translation and rotation
        Translation2d translationRequest = new Translation2d(requestedSpeeds.vxMetersPerSecond, requestedSpeeds.vyMetersPerSecond);
        Rotation2d rotationRequest = new Rotation2d(requestedSpeeds.omegaRadiansPerSecond);

        Translation2d currentTranslation = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        Rotation2d currentRotation = new Rotation2d(currentSpeeds.omegaRadiansPerSecond);

        // Limit speeds
        // double requestedSpeed = translationRequest.getNorm();
        // if (Math.abs(requestedSpeed) > maxTranslationalVelocity) {
        //     requestedSpeed = Math.signum(requestedSpeed) * maxTranslationalVelocity;
        //     translationRequest = new Translation2d(requestedSpeed, translationRequest.getAngle());
        // }

        // double requestedRotationalSpeed = rotationRequest.getRadians();
        // if (Math.abs(requestedRotationalSpeed) > maxAngularVelocity) {
        //     requestedRotationalSpeed = Math.signum(requestedRotationalSpeed) * maxAngularVelocity;
        //     rotationRequest = new Rotation2d(requestedRotationalSpeed);
        // }

        // Find acceleration (ms^-2), (rad/s^2)
        // Translation2d translationalAcceleration = translationRequest.minus(currentTranslation).times(0.02); // (Request-Current)/Time
        // Rotation2d rotationalAcceleration = rotationRequest.minus(currentRotation).times(0.02); // (Request-Current)/Time

        // Calculate net acceleration
        // netAcceleration = calculateNetAcceleration(translationalAcceleration, rotationalAcceleration, cg, robotMass, currentSpeeds.omegaRadiansPerSecond);

        // Calculate max acceleration

        Translation2d deltaV = translationRequest.minus(currentTranslation);

        // Translation2d maxDeltaV = calculateMaxAcceleration(new Translation2d()).times(0.02);
        Translation2d maxAccel = new Translation2d(1.0, deltaV.getAngle());

        Translation2d maxDeltaV = maxAccel.times(0.02);

        // Limit acceleration
        if (deltaV.getNorm() > maxDeltaV.getNorm()) {
            double shrinkCoefficient = maxDeltaV.getNorm() / deltaV.getNorm();
            translationRequest = currentTranslation.plus(deltaV.times(shrinkCoefficient));

        //     // double shrinkCoefficient = maxAcceleration / netAcceleration.getNorm();
        //     // Logger.getInstance().recordOutput("Shrink Coefficient", shrinkCoefficient);
        //     // translationalAcceleration.times(shrinkCoefficient);
        //     // // rotationalAcceleration.times(shrinkCoefficient);
        }

        Logger.getInstance().recordOutput("Current X", currentTranslation.getX());
        // Logger.getInstance().recordOutput("Acceleration X", translationalAcceleration.getX());
        // Logger.getInstance().recordOutput("Output X", currentTranslation.plus(translationalAcceleration).getX());

        System.out.println(translationRequest.getNorm());
        Logger.getInstance().recordOutput("Requested Speed", translationRequest.getNorm());
        // return new ChassisSpeeds(translationRequest.getX(), translationRequest.getY(), rotationRequest.getRadians());
        return new ChassisSpeeds(translationRequest.getX(), translationRequest.getY(), rotationRequest.getRadians());
        // return new ChassisSpeeds(translationRequest.getX(), translationRequest.getY(), rotationRequest.getRadians());
        // return new ChassisSpeeds(currentTranslation.getX() + translationalAcceleration.getX(), currentTranslation.getY() + translationRequest.getY(), 0);//currentRotation.getRadians() + rotationalAcceleration.getRadians());
    }

    public Translation2d getNetAcceleration() {
        return netAcceleration;
    }

    private Translation2d calculateNetAcceleration(Translation2d translationalAcceleration, Rotation2d rotationalAcceleration, Translation3d cg, double robotMass, double omegaRadiansPerSecond) {
        // Convert rotational accleration to force vector
        Translation2d toCG = cg.toTranslation2d();
        double force = rotationalAcceleration.getRadians() * robotMass * toCG.getNorm(); // alpha * mass * radius
        // Calculate the forces
        Translation2d forceFromRotation = new Translation2d(force, toCG.rotateBy(Rotation2d.fromDegrees(90)).getAngle());
        Translation2d forceFromTranslation = translationalAcceleration.times(robotMass); // F=ma
        Translation2d centripedialForce = new Translation2d(Math.pow(omegaRadiansPerSecond, 2)/toCG.getNorm(), toCG.unaryMinus().getAngle());
        // Sum the forces together
        // Translation2d netForce = forceFromRotation.plus(forceFromTranslation).plus(centripedialForce);
        Translation2d netForce = forceFromTranslation;
        // Find the acceleration
        Logger.getInstance().recordOutput("Force from Rotation", forceFromRotation.getNorm());
        Logger.getInstance().recordOutput("Force from Translation", forceFromTranslation.getNorm());
        Logger.getInstance().recordOutput("Acceleration", netForce.div(robotMass).div(0.02).getNorm());
        return netForce.div(robotMass).div(0.02);
    }

    private Translation2d calculateMaxAcceleration(Translation2d acceleration) {
        // TODO: Worst case senario

        // Determine which modules it would rotate on 
        List<Translation2d> modulePositions = List.of(DriveConstants.modulePositions);
        Translation2d closestModule = acceleration.nearest(modulePositions);
        modulePositions.remove(closestModule);
        modulePositions.remove(acceleration.nearest(modulePositions));
        Translation2d farModule = closestModule.nearest(modulePositions);

        // Flatten the problem into 2d (robots always tip over two wheels, never diagonally)
        if (closestModule.minus(farModule).getX() != 0) {
            // It's an X problem 
        } else {
            // It's a Y problem
        }

        return new Translation2d(1.0, new Rotation2d()); // ms^-2
    }
}
