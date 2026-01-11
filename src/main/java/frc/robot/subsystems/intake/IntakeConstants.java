package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import org.ironmaple.simulation.IntakeSimulation;

/**
 * Constants for the Intake subsystem.
 */
public final class IntakeConstants {

    private IntakeConstants() {}

    // Simulation constants
    public static final Distance WIDTH = Inches.of(20);
    public static final Distance LENGTH = Inches.of(8);
    public static final IntakeSimulation.IntakeSide ORIENTATION = IntakeSimulation.IntakeSide.BACK;
    public static final int MAX_CAPACITY = 8;
}
