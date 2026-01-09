package frc.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A utility class for working with poses.
 * Note: Use {@link FlippingUtil instead}.
 */
public final class PoseUtil {

    private PoseUtil() {}

    public static boolean isPoseTranslationNear(Pose2d pose1, Pose2d pose2, Distance distanceTolerance) {
        return (
            Math.abs(pose1.getX() - pose2.getX()) < distanceTolerance.in(Meters) &&
            Math.abs(pose1.getY() - pose2.getY()) < distanceTolerance.in(Meters)
        );
    }

    public static boolean isPoseRotationNear(Pose2d pose1, Pose2d pose2, Angle angleTolerance) {
        return MathUtil.isNear(
            pose1.getRotation().getRadians(),
            pose2.getRotation().getRadians(),
            angleTolerance.in(Radians),
            -Math.PI,
            Math.PI
        );
    }

    /**
     * Checks if two poses are within a specified tolerance of each other.
     * @param pose1 - The first pose.
     * @param pose2 - The second pose.
     * @param distanceTolerance - The maximum allowed distance between the two poses.
     * @param angleTolerance - The maximum allowed angle difference between the two poses.
     * @return true if the poses are within the specified tolerance, false otherwise.
     */
    public static boolean isNear(Pose2d pose1, Pose2d pose2, Distance distanceTolerance, Angle angleTolerance) {
        return isNear(pose1, pose2, distanceTolerance.in(Meters), angleTolerance.in(Radians));
    }

    /**
     * Checks if two poses are within a specified tolerance of each other.
     * @param pose1 - The first pose.
     * @param pose2 - The second pose.
     * @param distanceTolerance - The maximum allowed distance between the two poses.
     * @param angleTolerance - The maximum allowed angle difference between the two poses.
     * @return true if the poses are within the specified tolerance, false otherwise.
     */
    public static boolean isNear(
        Pose2d pose1,
        Pose2d pose2,
        double distanceToleranceMeters,
        double angleToleranceRadians
    ) {
        return (
            Math.abs(pose1.getX() - pose2.getX()) < distanceToleranceMeters &&
            Math.abs(pose1.getY() - pose2.getY()) < distanceToleranceMeters &&
            MathUtil.isNear(
                pose1.getRotation().getRadians(),
                pose2.getRotation().getRadians(),
                angleToleranceRadians,
                -Math.PI,
                Math.PI
            )
        );
    }

    public static boolean isVelocityNear(
        LinearVelocity velocity1,
        LinearVelocity velocity2,
        LinearVelocity velocityTolerance
    ) {
        return (
            Math.abs(velocity1.in(MetersPerSecond) - velocity2.in(MetersPerSecond)) <
            velocityTolerance.in(MetersPerSecond)
        );
    }

    /**
     * Checks if two poses and their velocities are within a specified tolerance of each other.
     * @param pose1 - The first pose.
     * @param pose2 - The second pose.
     * @param velocity1 - The velocity of the first pose.
     * @param velocity2 - The velocity of the second pose.
     * @param distanceTolerance - The maximum allowed distance between the two poses.
     * @param angleTolerance - The maximum allowed angle difference between the two poses.
     * @param velocityTolerance - The maximum allowed velocity difference between the two poses.
     * @return true if the poses and velocities are within the specified tolerance, false otherwise.
     */
    public static boolean isPosesAndVelocityNear(
        Pose2d pose1,
        Pose2d pose2,
        LinearVelocity velocity1,
        LinearVelocity velocity2,
        Distance distanceTolerance,
        Angle angleTolerance,
        LinearVelocity velocityTolerance
    ) {
        return (
            isNear(pose1, pose2, distanceTolerance, angleTolerance) &&
            Math.abs(velocity1.in(MetersPerSecond) - velocity2.in(MetersPerSecond)) <
            velocityTolerance.in(MetersPerSecond)
        );
    }

    public static double applyX(double x) {
        return shouldFlip() ? FieldConstants.fieldLength.in(Meters) - x : x;
    }

    public static double applyY(double y) {
        return shouldFlip() ? FieldConstants.fieldLength.in(Meters) - y : y;
    }

    public static Translation2d apply(Translation2d translation) {
        return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
    }

    public static Rotation2d apply(Rotation2d rotation) {
        return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
    }

    public static Pose2d apply(Pose2d pose) {
        return shouldFlip() ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation())) : pose;
    }

    public static Translation3d apply(Translation3d translation) {
        return new Translation3d(applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
    }

    public static Rotation3d apply(Rotation3d rotation) {
        return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
    }

    public static Pose3d apply(Pose3d pose) {
        return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
    }

    /**
     * Checks if the robot should flip its pose based on the current alliance color.
     * @return true if the robot is on the red alliance, false otherwise.
     */
    public static boolean shouldFlip() {
        return (
            DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        );
    }
}
