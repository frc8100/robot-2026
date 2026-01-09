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

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;
import frc.robot.subsystems.vision.VisionSim.NeuralDetectorSimPipeline;
import frc.util.VelocityNoiseGenerator;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;

/**
 * IO implementation for vision sim using PhotonVision simulator.
 * Currently only supports object detection.
 */
public class VisionIOPhotonSim extends VisionIOPhotonVision {

    private final PhotonCameraSim cameraSim;

    /**
     * The neural detector pipelines. Can be null if not used.
     */
    private final NeuralDetectorSimPipeline[] pipelines;

    private final Swerve swerveSubsystem;

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Map<GamePieceObservationType, List<GamePieceObservation>> gamePieceObservationsByType = new EnumMap<>(
        GamePieceObservationType.class
    );

    private VisionConstants.CameraPipelines currentPipeline = VisionConstants.CameraPipelines.getDefault();

    // Noise
    private final VelocityNoiseGenerator.PoseVelocityNoiseGenerator poseNoiseGenerator =
        new VelocityNoiseGenerator.PoseVelocityNoiseGenerator(
            Centimeters.of(0.35),
            Centimeters.of(0.7),
            Degrees.of(0.5),
            Degrees.of(1.0)
        );

    /**
     * Creates a new VisionIOPhotonVisionSim.
     * @param name - The name of the camera.
     * @param poseSupplier - Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonSim(
        String name,
        Transform3d robotToCamera,
        SimCameraProperties cameraProperties,
        Swerve swerveSubsystem,
        NeuralDetectorSimPipeline[] pipelines
    ) {
        super(name, robotToCamera);
        // Add sim camera
        cameraSim = new PhotonCameraSim(camera, cameraProperties, VisionConstants.aprilTagLayout);
        VisionSim.getVisionSim().addCamera(cameraSim, robotToCamera);

        this.swerveSubsystem = swerveSubsystem;

        this.robotPoseSupplier = swerveSubsystem::getActualPose;
        this.pipelines = pipelines;

        // Init map
        for (GamePieceObservationType type : GamePieceObservationType.values()) {
            gamePieceObservationsByType.put(type, new ArrayList<>());
        }
    }

    @Override
    public void setPipeline(VisionConstants.CameraPipelines pipeline) {
        super.setPipeline(pipeline);
        currentPipeline = pipeline;
    }

    /**
     * @return Whether to run PhotonVision's {@link org.photonvision.simulation.VisionSystemSim#update}.
     * When false, only object detection is simulated.
     */
    public boolean shouldSimulatePhoton() {
        // TODO: does pipeline set work if simulation is not iterated?
        return currentPipeline.equals(VisionConstants.CameraPipelines.APRILTAG);
    }

    private static Field nextNTEntryTimeField = null;

    /**
     * @return The Field object for the {@link PhotonCameraSim#nextNTEntryTime} field. Can be null if an error occurred.
     */
    private static Field getNextNTEntryTimeField() {
        if (nextNTEntryTimeField == null) {
            try {
                // Use reflection to access private field
                nextNTEntryTimeField = PhotonCameraSim.class.getDeclaredField("nextNTEntryTime");
                nextNTEntryTimeField.setAccessible(true); // NOSONAR
            } catch (NoSuchFieldException e) {
                e.printStackTrace();
            }
        }

        return nextNTEntryTimeField;
    }

    /**
     * Gets the next entry time without consuming it.
     * See {@link PhotonCameraSim#consumeNextEntryTime()}.
     * @return The next entry time in microseconds, or empty if an error occurred or no entry time is available.
     */
    private Optional<Long> getNextEntryTimeNoConsume() {
        try {
            Field field = getNextNTEntryTimeField();
            if (field == null) {
                return Optional.empty();
            }

            // Store old value
            long oldNextNTEntryTime = (long) field.get(cameraSim);

            Optional<Long> nextEntryTimeMicrosecondsOpt = cameraSim.consumeNextEntryTime();

            // Restore old value
            field.set(cameraSim, oldNextNTEntryTime); // NOSONAR

            return nextEntryTimeMicrosecondsOpt;
        } catch (IllegalAccessException e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }

    private void updateDetection(VisionIOInputs inputs, long nextTimeMicroseconds) {
        Pose3d cameraPose = new Pose3d(robotPoseSupplier.get()).transformBy(robotToCamera);

        // Object detection

        // For each pipeline, get potential targets and see if they are visible
        for (NeuralDetectorSimPipeline pipeline : pipelines) {
            List<VisionTargetSim> potentialTargets = VisionSim.getVisionTargetSimFromNeuralPipeline(pipeline);

            for (VisionTargetSim target : potentialTargets) {
                if (cameraSim.canSeeTargetPose(cameraPose, target)) {
                    // Visible, add observation
                    Pose3d noisyPose = poseNoiseGenerator.applyNoise(
                        target.getPose(),
                        swerveSubsystem.getVelocityMagnitudeAsDouble() +
                        (robotPoseSupplier.get().minus(target.getPose().toPose2d()).getTranslation().getNorm() * 0.3)
                    );

                    gamePieceObservationsByType
                        .get(pipeline.type())
                        .add(new GamePieceObservation(nextTimeMicroseconds / 1e6, noisyPose, 0.0, pipeline.type()));
                }
            }
        }

        // Save game piece observations to inputs object
        for (GamePieceObservationType type : GamePieceObservationType.values()) {
            List<GamePieceObservation> observationsOfType = gamePieceObservationsByType.get(type);
            inputs.gamePieceObservationsByType[type.getArrayIndexForInputs()] =
                new GamePieceObservation[observationsOfType.size()];
            inputs.gamePieceObservationsByType[type.getArrayIndexForInputs()] = observationsOfType.toArray(
                new GamePieceObservation[observationsOfType.size()]
            );

            observationsOfType.clear();
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Optional<Long> nextEntryTimeMicrosecondsOpt = Optional.empty();

        if (shouldSimulatePhoton()) {
            // Use non-consuming method to allow PhotonVision to also use the entry time
            // nextEntryTimeMicrosecondsOpt = getNextEntryTimeNoConsume();

            // Update vision sim with current robot pose
            // TODO: Make this only do once with multiple cameras (this updates all cameras)
            VisionSim.getVisionSim().update(robotPoseSupplier.get());
            super.updateInputs(inputs);
            return;
        }

        // Use faster consuming method since PhotonVision won't use it
        nextEntryTimeMicrosecondsOpt = cameraSim.consumeNextEntryTime();

        if (!nextEntryTimeMicrosecondsOpt.isPresent()) {
            // No new data, clear old data
            inputs.gamePieceObservationsByType = new GamePieceObservation[GamePieceObservationType.values().length][0];
            return;
        }

        // Update detection
        updateDetection(inputs, nextEntryTimeMicrosecondsOpt.get());
    }
}
