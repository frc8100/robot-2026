package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.ironmaple.simulation.IntakeSimulation;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

/**
 * Constants for the Intake subsystem.
 */
public final class IntakeConstants {

    private IntakeConstants() {}

    public static final SmartMotorControllerConfig intakeMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(3)
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(30))
        .withClosedLoopRampRate(Seconds.of(0.2))
        .withOpenLoopRampRate(Seconds.of(0.2));

    public static final SmartMotorControllerConfig deployMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(36)
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(30))
        .withClosedLoopRampRate(Seconds.of(0.2))
        .withOpenLoopRampRate(Seconds.of(0.2));

    // Auto intake (constants for swerve)
    public static final Angle MAX_AUTO_INTAKE_YAW_ASSIST = Degrees.of(15);
    public static final Distance YAW_ASSIST_ACTIVATION_DISTANCE = Inches.of(24);
    public static final LinearVelocity AUTO_INTAKE_APPROACH_VELOCITY = MetersPerSecond.of(2);

    // Simulation constants
    public static final IntakeSimulation.IntakeSide ORIENTATION = IntakeSimulation.IntakeSide.FRONT;
    public static final Rotation2d ORIENTATION_AS_ROTATION = Rotation2d.kZero;

    public static final Distance WIDTH = Inches.of(20);
    public static final Distance HALF_OF_WIDTH = WIDTH.div(2);

    public static final Distance LENGTH = Inches.of(8);
    public static final Distance INTAKE_FORWARD_OFFSET = Inches.of(12);
    public static final Transform2d ROBOT_CENTER_TO_INTAKE_CENTER = new Transform2d(
        new Translation2d(INTAKE_FORWARD_OFFSET, Inches.of(0)),
        ORIENTATION_AS_ROTATION
    );

    public static final int MAX_CAPACITY = 24;

    // 2d arrangement of fuel in intake visualization
    public static final int ROWS_OF_FUEL = 5;
    public static final int COLUMNS_OF_FUEL = 5;
}
