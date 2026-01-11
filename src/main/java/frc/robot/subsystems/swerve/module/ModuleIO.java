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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {

        public SparkMotorControllerData driveMotorData = new SparkMotorControllerData();

        public boolean driveMotorConnected = true;

        /**
         * The feedforward voltage applied to the drive motor.
         */
        public double driveFFVolts = 0.0;

        public SparkMotorControllerData turnMotorData = new SparkMotorControllerData();

        public boolean turnMotorConnected = true;

        /**
         * The current absolute position of the turn module.
         */
        public Rotation2d turnAbsolutePosition = new Rotation2d();

        /**
         * Whether the CANCoder is connected or not.
         * On simulated hardware, this is always true.
         */
        public boolean canCoderConnected = true;

        /**
         * The odometry drive positions received this cycle, in radians.
         */
        public double[] odometryDrivePositionsRad = new double[] {};

        /**
         * The odometry turn positions received this cycle.
         */
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified open loop value (in volts). */
    public default void setDriveOpenLoop(double output) {}

    /** Run the turn motor at the specified open loop value (in volts). */
    public default void setTurnOpenLoop(double output) {}

    /** Run the drive motor at the specified velocity. Used internally. */
    public default void setDriveVelocity(SwerveModuleState desiredState, double driveFeedforwardVoltage) {}

    /** Run the turn motor to the specified rotation. Used internally. */
    public default void setTurnPosition(SwerveModuleState desiredState) {}

    /**
     * Sets the desired state for the module. Should update the turn and drive motors to reach the desired state.
     * @param desiredState - The desired state for the module.
     * @param currentRotation2d - The current rotation of the module. Used for optimization.
     * @param driveFeedforwardVoltage - The feedforward voltage to apply to the drive motor.
     */
    public default void setDesiredState(
        SwerveModuleState desiredState,
        Rotation2d currentRotation2d,
        double driveFeedforwardVoltage
    ) {
        setDriveVelocity(desiredState, driveFeedforwardVoltage);
        setTurnPosition(desiredState);
    }

    /**
     * Resets the motor relative angle encoder to the current absolute encoder (CANCoder on real hardware) position reading.
     */
    public default void syncMotorEncoderToAbsoluteEncoder() {}

    public default double getDriveFeedForwardVoltage(double feedforwardLinearForcesNewtons) {
        return 0.0;
    }
}
