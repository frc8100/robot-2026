package frc.util;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.vision.VisionConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

    private FieldConstants() {}

    /**
     * The length of the field, from one alliance station to the other.
     */
    public static final Distance fieldLength = Meters.of(VisionConstants.aprilTagLayout.getFieldLength());

    /**
     * The width of the field, from one side to the other.
     */
    public static final Distance fieldWidth = Meters.of(VisionConstants.aprilTagLayout.getFieldWidth());

    public static final Distance hubDiameter = Inches.of(41.7);

    /**
     * The diameter of fuel.
     */
    public static final Distance fuelDiameter = Centimeters.of(15);
    public static final Distance hubRadiusForShooting = hubDiameter.minus(fuelDiameter).div(2);

    public static final Distance hubTargetHeight = Inches.of(72).plus(Inches.of(4));

    /**
     * Where the starting line is on the field, measured from the inside of the starting line.
     */
    public static final Distance startingLineX = Inches.of(299.438);

    public static final double aprilTagWidth = Units.inchesToMeters(6.50);
    public static final int aprilTagCount = 22;
}
