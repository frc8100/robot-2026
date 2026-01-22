package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
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
        .withGearing(36)
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

    // Simulation constants
    public static final Distance WIDTH = Inches.of(20);
    public static final Distance LENGTH = Inches.of(8);
    public static final IntakeSimulation.IntakeSide ORIENTATION = IntakeSimulation.IntakeSide.FRONT;
    public static final int MAX_CAPACITY = 24;
}
