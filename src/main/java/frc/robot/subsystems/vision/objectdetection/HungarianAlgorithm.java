package frc.robot.subsystems.vision.objectdetection;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionIO.GamePieceObservation;
import frc.robot.subsystems.vision.objectdetection.GamePiecePoseEstimator.DetectionAssociatedWithTrackedTarget;
import java.util.ArrayList;
import java.util.List;

/**
 * Implementation of the Hungarian Algorithm for assigning detected objects to tracked targets.
 */
public class HungarianAlgorithm {

    /**
     * Maximum cost allowed for greedy assignment in meters.
     */
    private static final double MAX_COST_ALLOWED_FOR_GREEDY_ASSIGNMENT_METERS = Inches.of(6).in(Meters);

    /**
     * When the cost of assigning a detection to a tracked target is below this threshold, instantly assign them without checking other options.
     */
    private static final double INSTANT_ASSIGNMENT_COST_THRESHOLD_METERS = Inches.of(3).in(Meters);

    /**
     * Produce a cost matrix where rows are detections and columns are tracked targets.
     */
    // private Matrix<?, ?> getCostMatrix(
    //     List<GamePieceObservation> detections,
    //     List<TrackedVisionTarget> trackedTargets
    // ) {
    //     return null;
    // }

    /**
     * Calculate the cost of assigning a detection to a tracked target.
     * The cost is the Euclidean distance between the detection pose and the tracked target's estimated pose (x and y only).
     * @param detection - The game piece observation detection.
     * @param trackedTarget - The tracked vision target.
     * @return The cost of the assignment.
     */
    private double getCost(GamePieceObservation detection, TrackedVisionTarget trackedTarget) {
        Translation2d detectionTranslation = detection.pose();
        Translation2d targetTranslation = trackedTarget.getEstimatedPose();

        // Only compare x and y distances for cost calculation (ignore z)
        return Math.hypot(
            detectionTranslation.getX() - targetTranslation.getX(),
            detectionTranslation.getY() - targetTranslation.getY()
        );
    }

    /**
     * The result of assigning detections to tracked targets in {@link #assignDetectionsToTargets(List, List)}.
     * Users should not modify the lists directly.
     */
    public static class AssignmentResult {

        /**
         * The list of assignments of detections to tracked targets.
         */
        public final List<DetectionAssociatedWithTrackedTarget> assignments = new ArrayList<>();

        /**
         * The list of unassociated detections.
         */
        public final List<GamePieceObservation> unassociatedDetections = new ArrayList<>();
    }

    private final AssignmentResult cachedResult = new AssignmentResult();

    /**
     * Assign detections to tracked targets using a greedy algorithm based on cost.
     * @param detections - The list of game piece observations.
     * @param trackedTargets - The list of tracked vision targets.
     * @return A list of assignments of detections to tracked targets.
     */
    public AssignmentResult assignDetectionsToTargets(
        GamePieceObservation[] detections,
        List<TrackedVisionTarget> trackedTargets
    ) {
        cachedResult.assignments.clear();
        cachedResult.unassociatedDetections.clear();

        // No assignments possible
        if (trackedTargets.isEmpty()) {
            // All detections are unassociated if there are no tracked targets
            if (detections.length > 0) {
                for (GamePieceObservation detection : detections) {
                    cachedResult.unassociatedDetections.add(detection);
                }
            }

            return cachedResult;
        }

        // For every tracked target and detection, reset assignment flags
        for (TrackedVisionTarget target : trackedTargets) {
            target.hasBeenAssignedThisFrame = false;
        }

        // Greedy algorithm
        for (GamePieceObservation detection : detections) {
            TrackedVisionTarget bestTarget = null;
            double bestCost = Double.MAX_VALUE;

            for (TrackedVisionTarget trackedTarget : trackedTargets) {
                // Skip already assigned targets
                if (trackedTarget.hasBeenAssignedThisFrame) {
                    continue;
                }

                double cost = getCost(detection, trackedTarget);

                // Check if this is the best cost so far
                if (cost < bestCost) {
                    bestCost = cost;
                    bestTarget = trackedTarget;
                }

                // If the cost is below the instant assignment threshold, assign it immediately.
                if (bestCost <= INSTANT_ASSIGNMENT_COST_THRESHOLD_METERS) {
                    break;
                }
            }

            // Assign the best target if within allowed cost
            if (bestTarget != null && bestCost <= MAX_COST_ALLOWED_FOR_GREEDY_ASSIGNMENT_METERS) {
                cachedResult.assignments.add(new DetectionAssociatedWithTrackedTarget(detection, bestTarget));
            } else {
                // No suitable target found, mark as unassociated
                cachedResult.unassociatedDetections.add(detection);
            }
        }

        return cachedResult;
    }
}
