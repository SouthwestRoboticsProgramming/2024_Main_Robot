package com.swrobotics.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.config.CANAllocation;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

@SuppressWarnings("unused")
public class SwerveModule extends com.ctre.phoenix6.mechanisms.swerve.SwerveModule {

    private final String name; // For debugging
    private final SwerveModuleConstants constants;
    private final double driveRotationsPerMeter;
    
    private final DCMotorSim steerSim;
    private final DCMotorSim driveSim;
    private final TalonFXSimState steerSimState;
    private final TalonFXSimState driveSimState;
    private final CANcoderSimState encoderSimState;

    private SwerveModuleState targetState = new SwerveModuleState();

    public SwerveModule(SwerveModuleConstants constants, String name, String canbusName) {
        super(constants, canbusName);

        this.name = name;
        this.constants = constants;

        /* Calculate the ratio of drive motor rotation to meter on ground */
        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(constants.WheelRadius);
        driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;

        if (RobotBase.isSimulation()) {
            steerSim = new DCMotorSim(DCMotor.getFalcon500Foc(1), constants.SteerMotorGearRatio, constants.SteerInertia);
            driveSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), constants.DriveMotorGearRatio, constants.DriveInertia);

            driveSimState = getDriveMotor().getSimState();
            steerSimState = getSteerMotor().getSimState();
            encoderSimState = getCANcoder().getSimState();

            steerSimState.Orientation = constants.SteerMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            driveSimState.Orientation = constants.DriveMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        } else {
            steerSim = null;
            driveSim = null;
            steerSimState = null;
            driveSimState = null;
            encoderSimState = null;
        }
    }

    @Override
    public void apply(SwerveModuleState state, DriveRequestType driveRequestType) {
        apply(state, driveRequestType, SteerRequestType.MotionMagic);
    }
    
    @Override
    public void apply(SwerveModuleState state, DriveRequestType driveRequestType, SteerRequestType steerRequestType) {
        targetState = SwerveModuleState.optimize(state, getCachedPosition().angle);
        super.apply(state, driveRequestType, steerRequestType);
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    @Override
    public SwerveModulePosition getPosition(boolean refresh) {
        SwerveModulePosition pos = super.getPosition(refresh);

        // Make a copy since super.getPosition() always returns the same
        // instance of SwerveModulePosition
        return new SwerveModulePosition(
                pos.distanceMeters,
                pos.angle
        );
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveMotor().getVelocity().getValue() / driveRotationsPerMeter, Rotation2d.fromRotations(getSteerMotor().getPosition().getValue()));
    }

    /**
     * Updates the simulated version of the module.
     * <p>
     * Only call this in {@code simulationPeriodic()}
     * 
     *  @param supplyVoltage How much voltage the module is recieving from the battery
     */
    public void updateSim(double dtSeconds, double supplyVoltage) {

        steerSim.setInputVoltage(steerSimState.getMotorVoltage());
        driveSim.setInputVoltage(driveSimState.getMotorVoltage());

        driveSim.update(dtSeconds);
        steerSim.update(dtSeconds);

        steerSimState.setSupplyVoltage(supplyVoltage);
        driveSimState.setSupplyVoltage(supplyVoltage);
        encoderSimState.setSupplyVoltage(supplyVoltage);

        steerSimState.setRawRotorPosition(steerSim.getAngularPositionRotations() * constants.SteerMotorGearRatio);
        steerSimState.setRotorVelocity(steerSim.getAngularVelocityRPM() / 60.0 * constants.SteerMotorGearRatio);

        // /* CANcoders see the mechanism, so don't account for the steer gearing */
        encoderSimState.setRawPosition(steerSim.getAngularPositionRotations());
        encoderSimState.setVelocity(steerSim.getAngularVelocityRPM() / 60.0);

        driveSimState.setRawRotorPosition(driveSim.getAngularPositionRotations() * constants.DriveMotorGearRatio);
        driveSimState.setRotorVelocity(driveSim.getAngularVelocityRPM() / 60.0 * constants.DriveMotorGearRatio);
    }

    /** Describes settings needed to create a swerve module */
    public record Info( // Not directly used in SwerveModule but used to create the SwerveModuleConstants object
            int driveId, int turnId, int encoderId,
            Translation2d position,
            NTEntry<Double> offset,
            String name) {
        public Info(CANAllocation.SwerveIDs ids, double x, double y, NTEntry<Double> offset, String name) {
            this(ids.drive, ids.turn, ids.encoder,
                    new Translation2d(x, y),
                    offset, name);
        }
    }
}
