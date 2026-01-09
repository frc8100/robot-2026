package frc.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.photonvision.estimation.OpenCVHelp;

/**
 * Utility class for vision-related functions.
 */
public class VisionUtil {

    private VisionUtil() {}

    /**
     * Estimates the Pose2d of a detected object on the field using Limelight data.
     * @param robotPose - Supplies the current robot pose (field-relative)
     * @param robotToCamera - Transform from the robot center to the camera
     * @param txRad - Horizontal offset from Limelight (degrees, +right)
     * @param tyRad - Vertical offset from Limelight (degrees, +down)
     * @param targetHeight - Height of the target above the floor (≈ 0 if lying flat)
     * @return The estimated Pose3d of the object on the field
     */
    public static Pose3d estimateTargetPose3d(
        Pose2d robotPose,
        Transform3d robotToCamera,
        double txRad,
        double tyRad,
        Distance targetHeight
    ) {
        // Compute distance from camera to target (flat)
        double totalPitchRad = -robotToCamera.getRotation().getY() + tyRad;
        double distanceToTargetMeters = (robotToCamera.getZ() - targetHeight.in(Meters)) / Math.tan(totalPitchRad);

        // Compute target position relative to camera
        double xCamMeters = distanceToTargetMeters * Math.cos(txRad);
        double yCamMeters = distanceToTargetMeters * Math.sin(txRad);
        Translation3d targetCam = new Translation3d(xCamMeters, yCamMeters, 0.0);

        // Transform camera-relative to robot-relative
        Translation3d targetRobot = robotToCamera.plus(new Transform3d(targetCam, Rotation3d.kZero)).getTranslation();

        // Get robot pose and apply transform robot-relative to field
        Translation2d targetFieldTranslation = robotPose
            .transformBy(new Transform2d(new Translation2d(targetRobot.getX(), targetRobot.getY()), Rotation2d.kZero))
            .getTranslation();

        // TODO: investigate this
        // OpenCVHelp.solvePNP_SQPNP(null, null, null, null);

        // Return the target pose
        return new Pose3d(new Pose2d(targetFieldTranslation, Rotation2d.kZero));
    }
}
