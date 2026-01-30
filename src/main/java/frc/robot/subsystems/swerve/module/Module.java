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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.CANIdConstants;
import frc.robot.subsystems.CANIdAlert;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.Logger;

/**
 * A single swerve module, including a drive motor and a turn motor.
 * Includes an abstracted IO interface for easy swapping of hardware/sim.
 */
public class Module {

    /**
     * The IO interface for a swerve module.
     * Get inputs from the module and set outputs to the module.
     */
    private final ModuleIO io;

    /** The inputs to the module. */
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    /** The index of the module. */
    public final int index;

    /**
     * The name of the module on the dashboard.
     */
    public final String dashboardInputsTableName;

    // Alerts for disconnected motors
    private final CANIdAlert driveDisconnectedAlert;
    private final CANIdAlert turnDisconnectedAlert;
    private final CANIdAlert cancoderDisconnectedAlert;

    /** The odometry positions received this cycle. This is processed in {@link SparkSwerveOdometryThread}. */
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    /** Creates a new module with the given IO and index. */
    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        this.dashboardInputsTableName = "Drive/Module" + Integer.toString(index);

        CANIdConstants.SwerveModuleCanIDs canIds = CANIdConstants.getModuleCANIdsFromIndex(index);

        driveDisconnectedAlert = new CANIdAlert(canIds.driveMotorID(), "DriveMotor" + Integer.toString(index));
        turnDisconnectedAlert = new CANIdAlert(canIds.angleMotorID(), "TurnMotor" + Integer.toString(index));
        cancoderDisconnectedAlert = new CANIdAlert(canIds.canCoderID(), "CANCoder" + Integer.toString(index));
    }

    /** Updates the inputs to the module. */
    public void periodic() {
        // Update inputs
        io.updateInputs(inputs);
        Logger.processInputs(dashboardInputsTableName, inputs);

        // Calculate positions for odometry

        // All signals are sampled together
        int sampleCount = inputs.odometryDrivePositionsRad.length;
        odometryPositions = new SwerveModulePosition[sampleCount];

        // For each sample, convert the drive position to meters and pair it with the turn angle
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * SwerveConstants.WHEEL_RADIUS.in(Meters);
            Rotation2d angle = inputs.odometryTurnPositions[i];

            // Store the position
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.updateConnectionStatus(inputs.driveMotorConnected);
        turnDisconnectedAlert.updateConnectionStatus(inputs.turnMotorConnected);
        cancoderDisconnectedAlert.updateConnectionStatus(inputs.canCoderConnected);
    }

    /**
     * Runs the module with the specified setpoint state. Mutates the state to optimize it.
     * @param state - The desired state of the module.
     * @param driveFeedforwardVoltage - The feedforward voltage to apply to the drive motor.
     * @param angleFeedforwardVoltage - The feedforward voltage to apply to the angle motor.
     */
    public void runSetpoint(SwerveModuleState state, double driveFeedforwardVoltage, double angleFeedforwardVoltage) {
        // Apply setpoints
        io.setDesiredState(state, getAngle(), driveFeedforwardVoltage, angleFeedforwardVoltage);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new SwerveModuleState(), 0.0);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    /**
     * Resets the motor relative angle encoder to the current absolute encoder position reading.
     */
    public void syncMotorEncoderToAbsoluteEncoder() {
        io.syncMotorEncoderToAbsoluteEncoder();
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.turnAbsolutePosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.driveMotorData.positionAngle.in(Radians) * SwerveConstants.WHEEL_RADIUS.in(Meters);
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveMotorData.velocity.in(RadiansPerSecond) * SwerveConstants.WHEEL_RADIUS.in(Meters);
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the module position. */
    public Angle getWheelRadiusCharacterizationPosition() {
        return inputs.driveMotorData.positionAngle;
    }

    /** Returns the module velocity. */
    public AngularVelocity getFFCharacterizationVelocity() {
        return inputs.driveMotorData.velocity;
    }

    /** Returns the current draw. */
    public Current getWheelSlippingCharacterization() {
        return inputs.driveMotorData.torqueCurrent;
    }
}
