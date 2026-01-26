// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

/**
 * The IO layer for a single vision camera.
 */
public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {

        /**
         * Whether the vision device is currently connected.
         */
        public boolean connected = false;
        // public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());

        /**
         * A queue of unread pose observations.
         */
        public PoseObservation[] poseObservations = new PoseObservation[0];

        /**
         * The tag ids used for the pose observations.
         */
        public int[] tagIds = new int[0];

        /**
         * A queue of unread game piece observations by type.
         * Type corresponds to {@link GamePieceObservationType#getArrayIndexForInputs()}.
         */
        public GamePieceObservation[][] gamePieceObservationsByType;

        public VisionIOInputs() {
            // Initialize the game piece observations array
            int typeCount = GamePieceObservationType.values().length;
            gamePieceObservationsByType = new GamePieceObservation[typeCount][0];
        }
    }

    @FunctionalInterface
    public interface RobotPoseAtTimestampSupplier {
        /**
         * Gets the robot pose at the given timestamp.
         * @param timestampSeconds - The timestamp in seconds.
         * @return The robot pose at the given timestamp.
         */
        public Optional<Pose2d> getRobotPoseAtTimestamp(double timestampSeconds);
    }

    /**
     * Represents the angle to a simple target, not used for pose estimation.
     * @param tx - The horizontal angle to the target.
     * @param ty - The vertical angle to the target.
     */
    // public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    /**
     * Represents a robot pose sample used for pose estimation.
     * @param timestamp - The timestamp of the observation.
     * @param pose - The observed field-relative pose of the robot.
     * @param ambiguity - The ambiguity of the observation (lower is better).
     * @param tagCount - The number of tags used in the observation.
     * @param averageTagDistance - The average distance to the tags used in the observation, in meters.
     * @param type - The type of observation.
     */
    public static record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance,
        PoseObservationType type
    ) {}

    public enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION,
    }

    /**
     * Represents a game piece observation used for game piece pose estimation.
     * @param timestampSeconds - The timestamp of the observation in seconds.
     * @param pose - The observed field-relative pose of the game piece.
     * @param ambiguity - The ambiguity of the observation (lower is better).
     * @param type - The type of game piece observed.
     */
    public static record GamePieceObservation(
        double timestampSeconds,
        Translation2d pose,
        double ambiguity,
        GamePieceObservationType type
    ) {}

    /**
     * Updates the inputs of the vision IO.
     */
    public default void updateInputs(VisionIOInputs inputs) {}

    /**
     * Switches the camera pipeline to the given pipeline.
     * @param pipelineToSwitchTo - The pipeline to switch to.
     */
    public default void setPipeline(VisionConstants.CameraPipelines pipelineToSwitchTo) {}
}
