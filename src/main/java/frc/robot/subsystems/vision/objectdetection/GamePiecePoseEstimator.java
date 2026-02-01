package frc.robot.subsystems.vision.objectdetection;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;
import frc.robot.subsystems.vision.VisionIO.GamePieceObservation;
import frc.robot.subsystems.vision.objectdetection.HungarianAlgorithm.AssignmentResult;
import frc.robot.subsystems.vision.objectdetection.TrackedVisionTarget.TrackedVisionTargetLoggedInfo;
import frc.util.FieldConstants;
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
    public static Pose2d getTargetRobotPoseToGamePiece(Pose2d robotPose, Translation2d gamePiecePose) {
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
        hasObservationsThisFrame = true;

        // Skip empty observations
        if (observations.length == 0) {
            return;
        }

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

    public record IntakeCostResult(
        double cost,
        Translation2d gamePiecePose,
        Pose2d robotPose,
        // debugging info
        double perpendicular,
        double parallel,
        double lateralError,
        double forwardError
    ) {}

    /**
     * Calculates the intake cost for a given robot pose and game piece pose.
     * @param robotPose - The field relative robot pose.
     * @param intakeCenter - The field relative intake center position.
     * @param gamePiecePose - The field relative game piece pose.
     * @return The intake cost result containing the cost and target robot pose.
     */
    private static IntakeCostResult getIntakeCost(
        Pose2d robotPose,
        Translation2d intakeCenter,
        Translation2d gamePiecePose
    ) {
        // Vector from intake center to game piece
        Translation2d delta = gamePiecePose.minus(intakeCenter);
        Translation2d deltaNormal = delta.rotateBy(
            robotPose.getRotation().plus(IntakeConstants.ORIENTATION_AS_ROTATION).unaryMinus()
        );

        // Decompose into perpendicular (straight ahead) and parallel (strafe) components
        double perpendicular = deltaNormal.getX();
        double parallel = deltaNormal.getY();

        double lateralError =
            parallel -
            MathUtil.clamp(
                parallel,
                -IntakeConstants.HALF_OF_WIDTH.in(Meters),
                IntakeConstants.HALF_OF_WIDTH.in(Meters)
            );
        double forwardError = Math.max(0.0, Math.abs(perpendicular) - FieldConstants.fuelRadius.in(Meters));

        // Quadratic cost (smooth, stable)
        double lateralCost = lateralError * lateralError;
        double forwardCost = forwardError * forwardError;

        // Cost for facing away from intake
        double facingCost = Math.max(0.0, -perpendicular);

        // Weight lateral error more heavily (intake alignment matters more)
        double cost = (4.0 * lateralCost) + (1.5 * forwardCost) + (0.5 * facingCost * facingCost);

        Pose2d targetPose = robotPose.plus(
            new Transform2d(deltaNormal.rotateBy(IntakeConstants.ORIENTATION_AS_ROTATION), Rotation2d.kZero)
        );

        // Apply yaw assist if close enough laterally and within activation distance
        if (
            Math.abs(lateralError) > 0.05 &&
            Math.abs(perpendicular) < IntakeConstants.YAW_ASSIST_ACTIVATION_DISTANCE.in(Meters)
        ) {
            double yawError = Math.atan2(parallel, perpendicular);
            double yawAdjustment = MathUtil.clamp(
                yawError,
                -IntakeConstants.MAX_AUTO_INTAKE_YAW_ASSIST.in(Radians),
                IntakeConstants.MAX_AUTO_INTAKE_YAW_ASSIST.in(Radians)
            );

            targetPose = new Pose2d(
                targetPose.getTranslation(),
                robotPose.getRotation().plus(new Rotation2d(yawAdjustment))
            );
        }

        return new IntakeCostResult(
            cost,
            gamePiecePose,
            targetPose,
            perpendicular,
            parallel,
            lateralError,
            forwardError
        );
    }

    /**
     * Gets the target robot pose to intake the nearest game piece of the given type.
     * @param type - The game piece observation type.
     * @param robotPoseSupplier - A supplier that provides the current robot pose.
     * @return The target robot pose to intake the nearest game piece of the given type. If no game pieces are observed, returns the current robot pose.
     */
    public Pose2d getIntakeTargetPose(GamePieceObservationType type, Supplier<Pose2d> robotPoseSupplier) {
        IntakeCostResult lowestCostResult = null;

        List<TrackedVisionTarget> targetsOfType = trackedTargetsByType.get(type);

        // Precompute position values
        Pose2d robotPose = robotPoseSupplier.get();
        Translation2d intakeCenter = Intake.getIntakeCenterPosition(robotPose);

        // Find the target pose with the lowest intake cost
        for (TrackedVisionTarget target : targetsOfType) {
            Translation2d gamePiecePose = target.getEstimatedPose();
            IntakeCostResult costResult = getIntakeCost(robotPose, intakeCenter, gamePiecePose);

            if (lowestCostResult == null || costResult.cost < lowestCostResult.cost) {
                lowestCostResult = costResult;
            }
        }

        // If no targets, return current robot pose
        if (lowestCostResult == null) {
            return robotPose;
        }

        Logger.recordOutput("Vision/LowestCostResult", lowestCostResult);

        return lowestCostResult.robotPose;
    }
}
