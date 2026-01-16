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

    public static final Distance hubTargetHeight = Inches.of(72);

    // static {
    //     Logger.recordOutput("Test/FieldLength", fieldLength);
    //     Logger.recordOutput("Test/fieldWidth", fieldWidth);
    // }

    /**
     * Where the starting line is on the field, measured from the inside of the starting line.
     */
    public static final Distance startingLineX = Inches.of(299.438);

    public static class Processor {

        private Processor() {}

        public static final Pose2d centerFace = new Pose2d(
            VisionConstants.aprilTagLayout.getTagPose(16).get().getX(),
            0,
            Rotation2d.fromDegrees(90)
        );
    }

    public static class Barge {

        private Barge() {}

        /**
         * The width of the net
         */
        public static final Distance netWidth = Inches.of(40.0);

        /**
         * The height of the net
         */
        public static final Distance netHeight = Inches.of(88.0);

        public static final Translation2d farCage = new Translation2d(Inches.of(345.428), Inches.of(286.779));
        public static final Translation2d middleCage = new Translation2d(Inches.of(345.428), Inches.of(242.855));
        public static final Translation2d closeCage = new Translation2d(Inches.of(345.428), Inches.of(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.inchesToMeters(3.125);
        public static final double shallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {

        private CoralStation() {}

        /**
         * The length of the station, from the back of the station to the front.
         */
        public static final Distance stationLength = Inches.of(79.750);

        public static final Pose2d rightCenterFace = new Pose2d(
            Inches.of(33.526),
            Inches.of(25.824),
            Rotation2d.fromDegrees(144.011 - 90)
        );

        public static final Pose2d leftCenterFace = new Pose2d(
            rightCenterFace.getMeasureX(),
            fieldWidth.minus(rightCenterFace.getMeasureY()),
            Rotation2d.fromRadians(-rightCenterFace.getRotation().getRadians())
        );
    }

    public static final double aprilTagWidth = Units.inchesToMeters(6.50);
    public static final int aprilTagCount = 22;
}
