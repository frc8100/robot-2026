package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;
import frc.robot.subsystems.vision.VisionIO.GamePieceObservation;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class GamePiecePoseEstimatorSimpleOld {

    /**
     * The latest observed game piece poses by type.
     */
    private final Map<GamePieceObservationType, List<Pose3d>> latestGamePiecePoses = new EnumMap<>(
        GamePieceObservationType.class
    );

    /**
     * Constructs a new GamePiecePoseEstimator.
     */
    public GamePiecePoseEstimatorSimpleOld() {
        // Initialize the latestGamePiecePoses map with empty lists for each observation type
        for (GamePieceObservationType type : GamePieceObservationType.values()) {
            latestGamePiecePoses.put(type, new ArrayList<>());
            hasGamePieceBeenCleared.put(type, false);
        }
    }

    /**
     * Gets the latest observed game piece poses for the given type.
     * @param type - The game piece observation type.
     * @return A list of the latest observed game piece poses for the given type. If no poses have been observed, returns an empty list.
     */
    public List<Pose3d> getLatestGamePiecePoses(GamePieceObservationType type) {
        return latestGamePiecePoses.getOrDefault(type, List.of());
    }

    /**
     * Gets the latest observed game piece poses as 2D poses for the given type.
     * @param type - The game piece observation type.
     * @return A list of the latest observed game piece poses as 2D poses for the given type. If no poses have been observed, returns an empty list.
     */
    public List<Pose2d> getLatestGamePiecePosesAs2d(GamePieceObservationType type) {
        List<Pose3d> poses3d = getLatestGamePiecePoses(type);

        return poses3d.stream().map(Pose3d::toPose2d).toList();
    }

    /**
     * Gets the nearest observed game piece pose of the given type to the reference pose.
     * @param type - The game piece observation type.
     * @param referencePoseSupplier - A supplier that provides the reference pose.
     * @return An Optional containing the nearest observed game piece pose of the given type to the reference pose, or an empty Optional if no poses have been observed.
     */
    public Optional<Pose3d> getNearestGamePiecePose(
        GamePieceObservationType type,
        Supplier<Pose2d> referencePoseSupplier
    ) {
        List<Pose3d> poses = getLatestGamePiecePoses(type);

        // Return empty if no poses are available
        if (poses.isEmpty()) {
            return Optional.empty();
        }

        Pose2d referencePose = referencePoseSupplier.get();

        return Optional.of(
            Collections.min(
                poses,
                Comparator.comparingDouble(pose ->
                    pose.toPose2d().getTranslation().getDistance(referencePose.getTranslation())
                )
            )
        );
    }

    private final Map<GamePieceObservationType, Boolean> hasGamePieceBeenCleared = new EnumMap<>(
        GamePieceObservationType.class
    );

    /**
     * Updates the estimator with new game piece observations.
     * @param observations - An array of game piece observations.
     */
    public void updateWithObservations(GamePieceObservation[] observations) {
        // Skip empty observations
        if (observations.length == 0) {
            return;
        }
        // Loop over game piece observations
        hasGamePieceBeenCleared.clear();

        for (GamePieceObservation gamePieceObservation : observations) {
            latestGamePiecePoses.putIfAbsent(gamePieceObservation.type(), new ArrayList<>());

            // Clear previous poses if this is the first observation of this type in this cycle
            if (Boolean.FALSE.equals(hasGamePieceBeenCleared.getOrDefault(gamePieceObservation.type(), false))) {
                List<Pose3d> posesList = latestGamePiecePoses.get(gamePieceObservation.type());
                posesList.clear();

                // Mark as cleared
                hasGamePieceBeenCleared.put(gamePieceObservation.type(), true);
            }

            // Add latest pose
            latestGamePiecePoses.get(gamePieceObservation.type()).add(gamePieceObservation.pose());
        }
    }

    /**
     * Logs the latest observed game piece poses by type.
     */
    public void logGamePiecePoses() {
        // For each game piece type, log the latest observed poses
        for (Map.Entry<GamePieceObservationType, List<Pose3d>> entry : latestGamePiecePoses.entrySet()) {
            GamePieceObservationType type = entry.getKey();
            List<Pose3d> poses = entry.getValue();

            Logger.recordOutput(
                "Vision/GamePieces/" + type.className + "/LatestPoses",
                poses.toArray(new Pose3d[poses.size()])
            );
        }
    }
}
