package com.swrobotics.robot.subsystems.swerve.modules;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModule3 extends com.ctre.phoenix6.mechanisms.swerve.SwerveModule {

    private final SwerveModuleConstants constants;
    
    private final DCMotorSim steerSim;
    private final DCMotorSim driveSim;

    private SwerveModuleState targetState = new SwerveModuleState();
    private SwerveModulePosition pos = new SwerveModulePosition();

    public SwerveModule3(SwerveModuleConstants constants, String canbusName) {
        super(constants, canbusName);

        this.constants = constants;

        if (RobotBase.isSimulation()) {
            steerSim = new DCMotorSim(DCMotor.getFalcon500Foc(1), constants.SteerMotorGearRatio, constants.SteerInertia);
            driveSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), constants.DriveMotorGearRatio, constants.DriveInertia);
        } else {
            steerSim = null;
            driveSim = null;
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
        var state = getCurrentState();
        pos = new SwerveModulePosition(pos.distanceMeters + state.speedMetersPerSecond * 0.02, state.angle);
        return pos;
    }

    /**
     * Updates the simulated version of the module.
     * <p>
     * Only call this in {@code simulationPeriodic()}
     * 
     *  @param supplyVoltage How much voltage the module is recieving from the battery
     */
    public void updateSim(double dtSeconds, double supplyVoltage) {
        TalonFXSimState steerSimState = getSteerMotor().getSimState();
        TalonFXSimState driveSimState = getDriveMotor().getSimState();
        CANcoderSimState encoderSimState = getCANcoder().getSimState();

        steerSimState.Orientation = constants.SteerMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        driveSimState.Orientation = constants.DriveMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;

        steerSim.setInputVoltage(steerSimState.getMotorVoltage());
        driveSim.setInputVoltage(driveSimState.getMotorVoltage());

        driveSim.update(dtSeconds);
        steerSim.update(dtSeconds);

        steerSimState.setSupplyVoltage(supplyVoltage);
        driveSimState.setSupplyVoltage(supplyVoltage);
        encoderSimState.setSupplyVoltage(supplyVoltage);


        steerSimState.setRawRotorPosition(steerSim.getAngularPositionRotations() * constants.SteerMotorGearRatio);
        steerSimState.setRotorVelocity(steerSim.getAngularVelocityRPM() / 60.0 * constants.SteerMotorGearRatio);

        /* CANcoders see the mechanism, so don't account for the steer gearing */
        encoderSimState.setRawPosition(steerSim.getAngularPositionRotations());
        encoderSimState.setVelocity(steerSim.getAngularVelocityRPM() / 60.0);

        driveSimState.setRawRotorPosition(driveSim.getAngularPositionRotations() * constants.DriveMotorGearRatio);
        driveSimState.setRotorVelocity(driveSim.getAngularVelocityRPM() / 60.0 * constants.DriveMotorGearRatio);
    }
}
