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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;
import frc.util.LimelightHelpers;
import frc.util.VisionUtil;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {

    /**
     * The name of this Limelight. Used for NetworkTables.
     */
    private final String name;

    private final Swerve swerveSubsystem;

    /**
     * Supplier for the current estimated rotation, used for MegaTag 2.
     * See {@link frc.util.LimelightHelpers#SetRobotOrientation} for format.
     * Should be a double[6] array: {yaw, yawRate, pitch, pitchRate, roll, rollRate} in degrees and degrees per second.
     */
    private final Supplier<double[]> rotationSupplier;

    private final Transform3d transformRobotToCamera;

    private final DoubleArrayPublisher orientationPublisher;

    private final DoubleSubscriber latencySubscriber;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;
    private final DoubleArraySubscriber rawDetectionsSubscriber;

    private final List<PoseObservation> poseObservations = new ArrayList<>();
    private final Map<GamePieceObservationType, List<GamePieceObservation>> gamePieceObservationsByType = new EnumMap<>(
        GamePieceObservationType.class
    );

    private final Set<Integer> tagIds = new HashSet<>();

    private VisionConstants.CameraPipelines currentPipeline = VisionConstants.CameraPipelines.getDefault();

    /**
     * Creates a new VisionIOLimelight.
     * @param name - The configured name of the Limelight.
     * @param rotationSupplier - Supplier for the current estimated rotation, used for MegaTag 2. See {@link #rotationSupplier} for details.
     */
    public VisionIOLimelight(String name, Transform3d transformRobotToCamera, Swerve swerveSubsystem) {
        this.name = name;
        this.rotationSupplier = swerveSubsystem::getOrientationToPublish;
        this.swerveSubsystem = swerveSubsystem;
        this.transformRobotToCamera = transformRobotToCamera;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
        rawDetectionsSubscriber = table.getDoubleArrayTopic("rawdetections").subscribe(new double[] {});

        // Initialize the map with empty lists for each observation type
        for (GamePieceObservationType type : GamePieceObservationType.values()) {
            gamePieceObservationsByType.put(type, new ArrayList<>());
        }
    }

    @Override
    public void setPipeline(VisionConstants.CameraPipelines pipelineToSwitchTo) {
        LimelightHelpers.setPipelineIndex(name, pipelineToSwitchTo.index);
        NetworkTableInstance.getDefault().flush();
    }

    /**
     * Reads a pose observation from a raw NetworkTables sample.
     * @param rawSample - The raw sample double array from NetworkTables.
     * @param type - The type of pose observation.
     */
    private void readPoseObservation(TimestampedDoubleArray rawSample, PoseObservationType type) {
        if (rawSample.value.length == 0) return;

        // Add tag IDs to set
        for (int i = 11; i < rawSample.value.length; i += 7) {
            tagIds.add((int) rawSample.value[i]);
        }

        poseObservations.add(
            new PoseObservation(
                // Timestamp, based on server timestamp of publish and latency
                rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
                // 3D pose estimate
                parsePose(rawSample.value),
                // Ambiguity or zeroed for MegaTag 2
                type == PoseObservationType.MEGATAG_1 && rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,
                // Tag count
                (int) rawSample.value[7],
                // Average tag distance
                rawSample.value[9],
                // Observation type
                type
            )
        );
    }

    private static final int VALUES_PER_DETECTION_ENTRY = 12;

    /**
     * Reads game piece observations from a raw NetworkTables sample.
     * @param rawSample - The raw sample double array from NetworkTables.
     * @param latencySeconds - The latency in seconds to compensate for.
     */
    private void readGamePieceObservations(TimestampedDoubleArray rawSample, double latencySeconds) {
        if (rawSample.value.length % VALUES_PER_DETECTION_ENTRY != 0) {
            // Invalid data lengths
            return;
        }

        int numDetections = rawSample.value.length / VALUES_PER_DETECTION_ENTRY;

        for (int i = 0; i < numDetections; i++) {
            // Starting index for this detection's data
            int baseIndex = i * VALUES_PER_DETECTION_ENTRY;

            // int classId = (int) extractArrayEntry(rawDetectionArray, baseIndex);
            // double txnc = extractArrayEntry(rawDetectionArray, baseIndex + 1);
            // double tync = extractArrayEntry(rawDetectionArray, baseIndex + 2);
            // double ta = extractArrayEntry(rawDetectionArray, baseIndex + 3);
            // double corner0_X = extractArrayEntry(rawDetectionArray, baseIndex + 4);
            // double corner0_Y = extractArrayEntry(rawDetectionArray, baseIndex + 5);
            // double corner1_X = extractArrayEntry(rawDetectionArray, baseIndex + 6);
            // double corner1_Y = extractArrayEntry(rawDetectionArray, baseIndex + 7);
            // double corner2_X = extractArrayEntry(rawDetectionArray, baseIndex + 8);
            // double corner2_Y = extractArrayEntry(rawDetectionArray, baseIndex + 9);
            // double corner3_X = extractArrayEntry(rawDetectionArray, baseIndex + 10);
            // double corner3_Y = extractArrayEntry(rawDetectionArray, baseIndex + 11);

            double timestampSeconds = rawSample.timestamp * 1.0e-6 - latencySeconds;

            GamePieceObservationType type = GamePieceObservationType.fromClassID(
                (int) LimelightHelpers.extractArrayEntry(rawSample.value, baseIndex)
            );

            // Estimate pose
            Translation2d pose = VisionUtil.estimateTargetPose2d(
                swerveSubsystem.poseEstimator.sampleAt(timestampSeconds).orElse(swerveSubsystem.getPose()),
                transformRobotToCamera,
                // TODO: should tx or ty be inverted?
                Units.degreesToRadians(LimelightHelpers.extractArrayEntry(rawSample.value, baseIndex + 1)),
                Units.degreesToRadians(LimelightHelpers.extractArrayEntry(rawSample.value, baseIndex + 2)),
                type.heightOffFloor
            );

            // Store observation
            gamePieceObservationsByType.get(type).add(new GamePieceObservation(timestampSeconds, pose, 0.0, type));
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected = RobotController.getFPGATime() - latencySubscriber.getLastChange() < 0.25;

        double latencySeconds = latencySubscriber.get() * 1.0e-3;

        // Only update MegaTag 2 if in the correct pipeline
        if (currentPipeline.equals(VisionConstants.CameraPipelines.APRILTAG)) {
            // Update orientation for MegaTag 2
            orientationPublisher.accept(rotationSupplier.get());
            // Increases network traffic but recommended by Limelight to flush after publishing
            NetworkTableInstance.getDefault().flush();
        }

        // Read new pose observations from NetworkTables
        for (TimestampedDoubleArray rawSample : megatag1Subscriber.readQueue()) {
            readPoseObservation(rawSample, PoseObservationType.MEGATAG_1);
        }
        for (TimestampedDoubleArray rawSample : megatag2Subscriber.readQueue()) {
            readPoseObservation(rawSample, PoseObservationType.MEGATAG_2);
        }

        // Read object detections
        for (TimestampedDoubleArray rawSample : rawDetectionsSubscriber.readQueue()) {
            readGamePieceObservations(rawSample, latencySeconds);
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }
        poseObservations.clear();

        // Save game piece observations to inputs object
        for (GamePieceObservationType type : GamePieceObservationType.values()) {
            List<GamePieceObservation> observationsOfType = gamePieceObservationsByType.get(type);

            // Clear existing array
            inputs.gamePieceObservationsByType[type.getArrayIndexForInputs()] =
                new GamePieceObservation[observationsOfType.size()];

            // Copy observations into inputs array
            observationsOfType.toArray(inputs.gamePieceObservationsByType[type.getArrayIndexForInputs()]);

            observationsOfType.clear();
        }

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
        tagIds.clear();
    }

    /**
     * Parses the 3D pose from a Limelight botpose array.
     */
    private static Pose3d parsePose(double[] rawLLArray) {
        return new Pose3d(
            rawLLArray[0],
            rawLLArray[1],
            rawLLArray[2],
            new Rotation3d(
                Units.degreesToRadians(rawLLArray[3]),
                Units.degreesToRadians(rawLLArray[4]),
                Units.degreesToRadians(rawLLArray[5])
            )
        );
    }
}
