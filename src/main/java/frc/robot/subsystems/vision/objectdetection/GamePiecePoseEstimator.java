package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;
import frc.robot.subsystems.vision.VisionIO.GamePieceObservation;
import frc.robot.subsystems.vision.objectdetection.HungarianAlgorithm.AssignmentResult;
import frc.robot.subsystems.vision.objectdetection.TrackedVisionTarget.TrackedVisionTargetLoggedInfo;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Implements a Kalman filter-based pose estimator for game pieces.
 */
public class GamePiecePoseEstimator {

    /**
     * Gets the target robot pose to a game piece.
     * Accounts for the intake not being at the front center of the robot.
     * @param robotPose - The current robot pose.
     * @param gamePiecePose - The game piece pose.
     * @return The target robot pose to the game piece.
     */
    public static Pose2d getTargetRobotPoseToGamePiece(Pose2d robotPose, Pose2d gamePiecePose) {
        // Calculate the angle from the robot to the game piece
        Rotation2d angleToGamePiece = new Rotation2d(
            gamePiecePose.getX() - robotPose.getX(),
            gamePiecePose.getY() - robotPose.getY()
        );

        // Calculate the target robot pose to the game piece
        // TODO: Account for intake being offset from robot center
        double targetRobotX = gamePiecePose.getX();
        double targetRobotY = gamePiecePose.getY();

        return new Pose2d(targetRobotX, targetRobotY, angleToGamePiece);
    }

    /**
     * A detection associated with a tracked target.
     * @param observation - The game piece observation.
     * @param trackedTarget - The tracked vision target.
     */
    public record DetectionAssociatedWithTrackedTarget(
        GamePieceObservation observation,
        TrackedVisionTarget trackedTarget
    ) {}

    /**
     * The Hungarian Algorithm instance for assignment of detections to tracked targets.
     */
    private final HungarianAlgorithm hungarianAlgorithm = new HungarianAlgorithm();

    /**
     * The tracked vision targets by type.
     */
    private final Map<GamePieceObservationType, List<TrackedVisionTarget>> trackedTargetsByType = new EnumMap<>(
        GamePieceObservationType.class
    );

    /**
     * Whether any observations have been processed this frame (50 hz robot loop).
     */
    private boolean hasObservationsThisFrame = false;

    /**
     * Constructs a new GamePiecePoseEstimator.
     */
    public GamePiecePoseEstimator() {
        // Initialize the map with empty lists for each observation type
        for (GamePieceObservationType type : GamePieceObservationType.values()) {
            trackedTargetsByType.put(type, new ArrayList<>());
        }
    }

    /**
     * Gets the latest observed game piece poses for the given type.
     * @param type - The game piece observation type.
     * @return A list of the latest observed game piece poses for the given type. If no poses have been observed, returns an empty list.
     */
    public List<Translation3d> getLatestGamePiecePoses(GamePieceObservationType type) {
        return trackedTargetsByType.get(type).stream().map(target -> target.getEstimatedPose3d()).toList();
    }

    /**
     * Gets the latest observed game piece poses as 2D poses for the given type.
     * @param type - The game piece observation type.
     * @return A list of the latest observed game piece poses as 2D poses for the given type. If no poses have been observed, returns an empty list.
     */
    public List<Translation2d> getLatestGamePiecePosesAs2d(GamePieceObservationType type) {
        return trackedTargetsByType.get(type).stream().map(target -> target.getEstimatedPose()).toList();
    }

    /**
     * Gets the nearest observed game piece pose of the given type to the reference pose.
     * @param type - The game piece observation type.
     * @param referencePoseSupplier - A supplier that provides the reference pose.
     * @return An Optional containing the nearest observed game piece pose of the given type to the reference pose, or an empty Optional if no poses have been observed.
     */
    public Optional<Translation2d> getNearestGamePiecePose(
        GamePieceObservationType type,
        Supplier<Pose2d> referencePoseSupplier
    ) {
        List<Translation2d> poses = getLatestGamePiecePosesAs2d(type);
        // Return empty if no poses are available
        if (poses.isEmpty()) {
            return Optional.empty();
        }

        Pose2d referencePose = referencePoseSupplier.get();

        Translation2d nearestPose = null;
        double nearestDistance = Double.MAX_VALUE;

        for (Translation2d pose : poses) {
            double distance = pose.getDistance(referencePose.getTranslation());
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestPose = pose;
            }
        }

        return Optional.ofNullable(nearestPose);
    }

    /**
     * Updates the estimator with new game piece observations.
     * @param observations - An array of game piece observations.
     * @param type - The type of game piece observations.
     */
    public void updateWithObservations(GamePieceObservation[] observations, GamePieceObservationType type) {
        // Skip empty observations
        if (observations.length == 0) {
            return;
        }

        hasObservationsThisFrame = true;

        List<TrackedVisionTarget> trackedTargetsOfType = trackedTargetsByType.get(type);

        updateWithAssociatedTargets(hungarianAlgorithm.assignDetectionsToTargets(observations, trackedTargetsOfType));
    }

    /**
     * Updates the tracked targets with the associated detections from the assignment result.
     * @param assignmentResult - The assignment result containing associated detections and unassociated detections.
     */
    private void updateWithAssociatedTargets(AssignmentResult assignmentResult) {
        // Update associated targets
        for (DetectionAssociatedWithTrackedTarget associatedDetection : assignmentResult.assignments) {
            GamePieceObservation observation = associatedDetection.observation;
            TrackedVisionTarget trackedTarget = associatedDetection.trackedTarget;

            // Update the tracked target with the new observation pose
            trackedTarget.addMeasurement(observation.pose());
        }

        // Create new tracked targets for unassociated detections
        for (GamePieceObservation observation : assignmentResult.unassociatedDetections) {
            TrackedVisionTarget newTarget = new TrackedVisionTarget(
                observation.type(),
                observation.timestampSeconds(),
                observation.pose()
            );

            trackedTargetsByType.get(observation.type()).add(newTarget);
        }
    }

    /**
     * Processes the observations to update hits and misses for tracked targets.
     * Should be called once per update cycle after observations have been processed.
     */
    public void processObservations() {
        if (!hasObservationsThisFrame) {
            // No observations this frame, nothing to process
            return;
        }

        // Reset for next frame
        hasObservationsThisFrame = false;

        for (List<TrackedVisionTarget> targets : trackedTargetsByType.values()) {
            // Update each target miss
            for (TrackedVisionTarget target : targets) {
                if (!target.hasBeenHitThisFrame) {
                    target.misses++;
                }
            }

            // Remove targets that should be deleted
            targets.removeIf(TrackedVisionTarget::shouldDelete);

            // Update each target
            for (TrackedVisionTarget target : targets) {
                target.hasBeenHitThisFrame = false;

                // Update target kalman filter
                target.update();
            }
        }
    }

    /**
     * Logs the latest observed game piece poses by type.
     */
    public void logGamePiecePoses() {
        if (!Constants.shouldLogAdditionalData()) {
            return;
        }

        // For each game piece type, log the latest observed poses
        for (Map.Entry<GamePieceObservationType, List<TrackedVisionTarget>> entry : trackedTargetsByType.entrySet()) {
            GamePieceObservationType type = entry.getKey();
            List<TrackedVisionTarget> targets = entry.getValue();

            // Log tracked target info
            Translation3d[] poses = new Translation3d[targets.size()];
            TrackedVisionTargetLoggedInfo[] infos = new TrackedVisionTargetLoggedInfo[targets.size()];

            for (int i = 0; i < targets.size(); i++) {
                poses[i] = targets.get(i).getEstimatedPose3d();
                infos[i] = targets.get(i).getLoggedInfo();
            }

            Logger.recordOutput("Vision/GamePieces/" + type.className + "/LatestPoses", poses);
            Logger.recordOutput("Vision/GamePieces/" + type.className + "/TrackedTargets", infos);
        }
    }
}
