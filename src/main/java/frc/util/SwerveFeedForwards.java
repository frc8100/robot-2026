package frc.util;

import static edu.wpi.first.units.Units.Meters;

import frc.robot.subsystems.swerve.SwerveConstants;
import java.util.function.BooleanSupplier;

public class SwerveFeedForwards {

    /**
     * Constants for linear force feedforward calculations.
     * See {@link #getLinearForcesFFVoltsFromRadPerSec} for details on calculating the feedforward voltage from linear forces.
     * Satisfies the equation: V_applied = kS * sign(velocityRadiansPerSecond) + kV * velocityRadiansPerSecond + kA * accelerationRadiansPerSecond2 + kF * calculatedLinearForceFFVolts.
     * kA has no effect in simulation as acceleration is not used in the calculation.
     * kA also has no effect in real hardware as the SparkMax does not support onboard acceleration feedforward.
     */
    public static record LinearForceFeedForwardConstants(double kS, double kV, double kA, double kF) {}

    /**
     * Constants for simple feedforward calculations.
     * Satisfies the equation: V_applied = kS * sign(velocityRadiansPerSecond) + kV * velocityRadiansPerSecond
     */
    public static record SimpleFeedForwardConstants(double kS, double kV) {}

    /**
     * Calculates the required voltage for the given linear forces and desired ground speed.
     * @param feedforwardLinearForcesNewtons - The desired linear forces in Newtons.
     * @param desiredSpeedRadPerSec - The desired ground speed in meters per second.
     * @return The required voltage to achieve the desired forces and speed.
     */
    public static double getLinearForcesFFVoltsFromRadPerSec(
        double feedforwardLinearForcesNewtons,
        double desiredSpeedRadPerSec
    ) {
        // Calculate ff voltage
        // Adapted from YAGSL SwerveDrive.drive

        // from the module configuration, obtain necessary information to calculate feed-forward
        // Warning: Will not work well if motor is not what we are expecting.

        // calculation:
        return SwerveConstants.driveGearbox.getVoltage(
            // Since: (1) torque = force * momentOfForce; (2) torque (on wheel) = torque (on motor) * gearRatio
            // torque (on motor) = force * wheelRadius / gearRatio
            (feedforwardLinearForcesNewtons * SwerveConstants.WHEEL_RADIUS.in(Meters)) /
            SwerveConstants.DRIVE_GEAR_RATIO,
            // Since: (1) linear velocity = angularVelocity * wheelRadius; (2) wheelVelocity = motorVelocity / gearRatio
            // motorAngularVelocity = linearVelocity / wheelRadius * gearRatio
            desiredSpeedRadPerSec * SwerveConstants.DRIVE_GEAR_RATIO
        );
    }

    /**
     * Calculates the required voltage for the given linear forces and desired ground speed.
     * @param feedforwardLinearForcesNewtons - The desired linear forces in Newtons.
     * @param desiredGroundSpeedMPS - The desired ground speed in meters per second.
     * @return The required voltage to achieve the desired forces and speed.
     */
    public static double getLinearForcesFFVoltsFromMPS(
        double feedforwardLinearForcesNewtons,
        double desiredGroundSpeedMPS
    ) {
        return getLinearForcesFFVoltsFromRadPerSec(
            feedforwardLinearForcesNewtons,
            desiredGroundSpeedMPS / SwerveConstants.WHEEL_RADIUS.in(Meters)
        );
    }

    // Drive Motor Characterization Values
    // TODO: Tune these values
    public static final LinearForceFeedForwardConstants driveFFConstantsReal = new LinearForceFeedForwardConstants(
        0.17388,
        0.13632,
        0,
        0
    );
    public static final LinearForceFeedForwardConstants driveFFConstantsSim = new LinearForceFeedForwardConstants(
        0.0752,
        0.0436,
        0,
        0.8849
    );

    // Angle Motor Characterization Values
    public static final SimpleFeedForwardConstants angleFFConstantsReal = new SimpleFeedForwardConstants(0.0, 0.0);
    public static final SimpleFeedForwardConstants angleFFConstantsSim = new SimpleFeedForwardConstants(0.0, 0.42514);

    // Instance feedforward constants
    private final SimpleFeedForwardConstants angleMotorFFConstants;
    private final LinearForceFeedForwardConstants driveMotorFFConstants;

    /**
     * Indicates whether the feedforwards are for simulation or real hardware.
     */
    public final boolean isSimulation;

    /**
     * Creates a new SwerveFeedForwards for the given simulation state.
     * @param isSimulationSupplier - Supplier that returns true if in simulation, false otherwise.
     */
    public SwerveFeedForwards(BooleanSupplier isSimulationSupplier) {
        this.isSimulation = isSimulationSupplier.getAsBoolean();

        if (isSimulation) {
            System.out.println("Using SIMULATION SwerveFeedForwards");
        } else {
            System.out.println("Using REAL SwerveFeedForwards");
        }

        // Select appropriate constants
        this.angleMotorFFConstants = isSimulation ? angleFFConstantsSim : angleFFConstantsReal;
        this.driveMotorFFConstants = isSimulation ? driveFFConstantsSim : driveFFConstantsReal;
    }

    /**
     * Calculates the required voltage for the given simple feedforward constants and desired velocity.
     * @param velocityRadPerSec - The desired velocity in radians per second.
     * @return The required voltage to achieve the desired velocity.
     */
    public double getAngleMotorFFVolts(double velocityRadPerSec) {
        return (
            angleMotorFFConstants.kS * Math.signum(velocityRadPerSec) + angleMotorFFConstants.kV * velocityRadPerSec
        );
    }

    /**
     * Calculates the required voltage for the given linear force feedforward constants, desired velocity, and feedforward linear forces.
     * If in real hardware, only the kF portion is calculated and returned as kS, kV, and kA are handled by the SparkMax.
     * @param desiredVelocityRadPerSec - The desired velocity in radians per second.
     * @param feedforwardLinearForcesNewtons - The desired linear forces in Newtons.
     * @return The required voltage to achieve the desired velocity and forces.
     */
    public double getDriveMotorFFVolts(double desiredVelocityRadPerSec, double feedforwardLinearForcesNewtons) {
        // If real, kS and kV are automatically handled by SparkMax
        if (!isSimulation) {
            return (
                driveMotorFFConstants.kF *
                getLinearForcesFFVoltsFromRadPerSec(feedforwardLinearForcesNewtons, desiredVelocityRadPerSec)
            );
        }

        return (
            driveMotorFFConstants.kS * Math.signum(desiredVelocityRadPerSec) +
            driveMotorFFConstants.kV * desiredVelocityRadPerSec +
            driveMotorFFConstants.kF *
            getLinearForcesFFVoltsFromRadPerSec(feedforwardLinearForcesNewtons, desiredVelocityRadPerSec)
        );
    }
}
