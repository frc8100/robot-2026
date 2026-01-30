// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.subsystems.swerve.SwerveConstants;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Simulator implementation of a module.
 */
public class ModuleIOSim implements ModuleIO {

    // Simulation components
    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    // Controllers
    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(
        SwerveConstants.driveSimKP,
        0,
        SwerveConstants.driveSimKD
    );
    private PIDController turnController = new PIDController(SwerveConstants.angleSimKP, 0, SwerveConstants.angleSimKD);

    // Voltages
    private final MutVoltage driveAppliedVolts = Volts.mutable(0.0);
    private final MutVoltage turnAppliedVolts = Volts.mutable(0.0);
    private final MutVoltage driveFFVolts = Volts.mutable(0.0);
    private final MutVoltage turnFFVolts = Volts.mutable(0.0);

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        this.driveMotor = moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT);
        this.turnMotor = moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(SwerveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT);

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts.mut_replace(
                driveFFVolts.in(Volts) +
                driveController.calculate(moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond)),
                Volts
            );
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts.mut_replace(
                turnFFVolts.in(Volts) +
                turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians()),
                Volts
            );
        } else {
            turnController.reset();
        }

        // Update simulation state
        driveMotor.requestVoltage(driveAppliedVolts);
        turnMotor.requestVoltage(turnAppliedVolts);

        // Update drive inputs
        inputs.driveMotorData.positionAngle.mut_replace(moduleSimulation.getDriveWheelFinalPosition());
        inputs.driveMotorData.velocity.mut_replace(moduleSimulation.getDriveWheelFinalSpeed());
        inputs.driveMotorData.appliedVolts.mut_replace(driveAppliedVolts);
        inputs.driveMotorData.torqueCurrent.mut_replace(moduleSimulation.getDriveMotorStatorCurrent());
        inputs.driveFFVolts = driveFFVolts.in(Volts);

        // Update turn inputs
        inputs.turnMotorData.positionAngle.mut_replace(moduleSimulation.getSteerRelativeEncoderPosition());
        inputs.turnMotorData.velocity.mut_replace(moduleSimulation.getSteerAbsoluteEncoderSpeed());
        inputs.turnMotorData.appliedVolts.mut_replace(turnAppliedVolts);
        inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();

        // Update odometry inputs
        inputs.odometryDrivePositionsRad = Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
        inputs.odometryTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts.mut_replace(output, Volts);
        driveFFVolts.mut_replace(Volts.zero());
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts.mut_replace(output, Volts);
        turnFFVolts.mut_replace(Volts.zero());
    }

    @Override
    public void setDriveVelocity(SwerveModuleState desiredState, double driveFeedforwardVoltage) {
        double velocityRadPerSec = desiredState.speedMetersPerSecond / SwerveConstants.WHEEL_RADIUS.in(Meters);

        driveClosedLoop = true;

        driveFFVolts.mut_replace(driveFeedforwardVoltage, Volts);

        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(SwerveModuleState desiredState, double angleFeedforwardVoltage) {
        turnFFVolts.mut_replace(angleFeedforwardVoltage, Volts);
        turnClosedLoop = true;
        turnController.setSetpoint(desiredState.angle.getRadians());
    }
}
