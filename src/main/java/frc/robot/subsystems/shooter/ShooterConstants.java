package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import org.ironmaple.simulation.IntakeSimulation;

/**
 * Constants for the Shooter subsystem.
 */
public final class ShooterConstants {

    private ShooterConstants() {}

    public static final Angle exitAngle = Degrees.of(60.0);

    public static final LinearAcceleration g = MetersPerSecondPerSecond.of(9.81);

    // Simulation constants

    public static final Transform3d positionFromRobotCenter = new Transform3d(
        new Translation3d(Inches.of(12.0), Inches.of(0.0), Inches.of(6)),
        Rotation3d.kZero
    );
}
