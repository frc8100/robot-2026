package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class ShooterCharacterization {

    private ShooterCharacterization() {}

    public static final AngularVelocity STARTING_ANGULAR_VELOCITY = RPM.of(300);
    public static final AngularVelocity ENDING_ANGULAR_VELOCITY = RPM.of(6000);
    public static final AngularVelocity VELOCITY_STEP = RPM.of(300);

    public static final Distance DISTANCE_STEP = Inches.of(6);

    public static final InterpolatingDoubleTreeMap metersPerSecondToShooterRadPerSecondMap =
        new InterpolatingDoubleTreeMap();

    static {
        // TODO: test and fill in values
        // metersPerSecondToShooterRadPerSecondMap.put(0.0, 0.0);
    }
}
