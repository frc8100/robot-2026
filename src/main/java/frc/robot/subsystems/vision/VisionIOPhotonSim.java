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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;
import frc.robot.subsystems.vision.VisionSim.NeuralDetectorSimPipeline;
import frc.util.CustomSimulationArena;
import frc.util.PoseUtil;
import frc.util.VelocityNoiseGenerator;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Ray;
import org.dyn4j.world.DetectFilter;
import org.dyn4j.world.World;
import org.dyn4j.world.result.RaycastResult;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.utils.mathutils.GeometryConvertor;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;

/**
 * IO implementation for vision sim using PhotonVision simulator.
 * Currently only supports object detection.
 */
public class VisionIOPhotonSim extends VisionIOPhotonVision {

    // public static final DetectFilter<Body, BodyFixture> SIM_DETECT_FILTER = new DetectFilter<>(
    //     false,
    //     false,

    // );

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

    /**
     * Updates the inputs with the simulated object detection data.
     */
    private void updateDetection(VisionIOInputs inputs, long nextTimeMicroseconds) {
        Pose3d cameraPose = new Pose3d(robotPoseSupplier.get()).transformBy(robotToCamera);

        // Object detection

        // For each pipeline, get potential targets and see if they are visible
        for (NeuralDetectorSimPipeline pipeline : pipelines) {
            List<VisionTargetSim> potentialTargets = VisionSim.getVisionTargetSimFromNeuralPipeline(pipeline);

            for (VisionTargetSim target : potentialTargets) {
                // Exclude targets that are not on field
                if (!PoseUtil.isPoseOnField(target.getPose().toPose2d())) {
                    continue;
                }

                // Exclude targets that are too far away
                double distance = robotPoseSupplier
                    .get()
                    .getTranslation()
                    .getDistance(target.getPose().toPose2d().getTranslation());
                if (distance > VisionConstants.SIM_MAX_DETECTION_DISTANCE.in(Meters)) {
                    continue;
                }

                // Check if target is visible
                if (!cameraSim.canSeeTargetPose(cameraPose, target)) {
                    continue;
                }

                // Check if target is obstructed by an obstacle in the arena
                if (SimulatedArena.getInstance() instanceof CustomSimulationArena) {
                    CustomSimulationArena arena = (CustomSimulationArena) SimulatedArena.getInstance();

                    World<Body> physicsWorld = arena.getPhysicsWorld();

                    Translation2d cameraToTarget = target
                        .getPose()
                        .toPose2d()
                        .relativeTo(cameraPose.toPose2d())
                        .getTranslation();

                    // Perform raycast from camera to target and check if it hits any obstacles
                    Iterator<RaycastResult<Body, BodyFixture>> raycastResults = physicsWorld.raycastIterator(
                        new Ray(
                            GeometryConvertor.toDyn4jVector2(cameraPose.getTranslation().toTranslation2d()),
                            GeometryConvertor.toDyn4jVector2(cameraToTarget)
                        ),
                        cameraToTarget.getNorm(),
                        // No filter, manually filter out sim robot
                        null
                    );

                    boolean obstructed = false;

                    int numberOfGamePiecesInPath = 0;

                    while (raycastResults.hasNext()) {
                        RaycastResult<Body, BodyFixture> result = raycastResults.next();

                        Body body = result.getBody();

                        // Check if hit body is sim robot, if so ignore
                        if (body instanceof AbstractDriveTrainSimulation) {
                            continue;
                        }

                        // Note: because of use of custom game piece simulation other than maple sim, this will not trigger
                        // if (body instanceof GamePieceOnFieldSimulation) {
                        //     numberOfGamePiecesInPath++;

                        //     // Allow up to 2 game pieces in path before considering it an obstruction, to allow for some error in target pose and robot pose
                        //     if (numberOfGamePiecesInPath <= 2) {
                        //         // TODO: refactor to reduce nesting
                        //         continue;
                        //     }
                        // }

                        // Otherwise, consider it an obstruction
                        obstructed = true;
                        break;
                    }

                    if (obstructed) {
                        continue;
                    }
                } else {
                    // If not using custom arena, print warning and continue without obstruction checking
                    System.out.println(
                        "Warning: VisionIOPhotonSim is not using CustomSimulationArena, so target obstruction is not being checked."
                    );
                }

                // Visible, add observation
                Pose3d noisyPose = poseNoiseGenerator.applyNoise(
                    target.getPose(),
                    swerveSubsystem.getVelocityMagnitudeAsDouble() +
                    (robotPoseSupplier.get().minus(target.getPose().toPose2d()).getTranslation().getNorm() * 0.3)
                );

                gamePieceObservationsByType
                    .get(pipeline.type())
                    .add(
                        new GamePieceObservation(
                            nextTimeMicroseconds / 1e6,
                            noisyPose.toPose2d().getTranslation(),
                            0.0,
                            pipeline.type()
                        )
                    );
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
