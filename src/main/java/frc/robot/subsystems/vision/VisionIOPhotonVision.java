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

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * IO implementation for real PhotonVision hardware.
 */
public class VisionIOPhotonVision implements VisionIO {

    /**
     * The PhotonCamera instance for this camera.
     */
    protected final PhotonCamera camera;

    /**
     * The transform from the robot to the camera.
     */
    protected final Transform3d robotToCamera;

    // Cached data for updateInputs
    private final Set<Short> tagIds = new HashSet<>();
    private final List<PoseObservation> poseObservations = new ArrayList<>();

    /**
     * Creates a new VisionIOPhotonVision.
     * @param name The configured name of the camera.
     * @param rotationSupplier The 3D position of the camera relative to the robot.
     */
    public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void setPipeline(VisionConstants.CameraPipelines pipelineToSwitchTo) {
        camera.setPipelineIndex(pipelineToSwitchTo.index);
    }

    // private Optional<GamePieceObservation> convertGamePieceObservation(PhotonTrackedTarget target, double timestampSeconds) {
    //     Optional<Pose2d> robotPoseOpt = robotPoseSupplier.getRobotPoseAtTimestamp(timestampSeconds);

    //     if (robotPoseOpt.isEmpty()) {
    //         return Optional.empty();
    //     }

    //     return Optional.of(new GamePieceObservation(
    //         timestampSeconds,
    //         VisionUtil.estimateTargetPose3d(
    //             robotPoseOpt.get(),
    //             robotToCamera,
    //             Degrees.of(target.getYaw()),
    //             Degrees.of(target.getPitch()),
    //             Meters.of(0)
    //         ),
    //         target.objDetectConf,
    //         GamePieceObservationType.fromClassID(target.objDetectId)
    //     ));
    // }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        // Read new camera observations
        List<PhotonPipelineResult> photonPipelineResults = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : photonPipelineResults) {
            // Add pose observation
            if (result.multitagResult.isPresent()) {
                // Multitag result
                MultiTargetPNPResult multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                // Calculate average tag distance
                double totalTagDistance = 0.0;
                for (PhotonTrackedTarget target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                // Add tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                // Add observation
                poseObservations.add(
                    new PoseObservation(
                        result.getTimestampSeconds(),
                        robotPose,
                        multitagResult.estimatedPose.ambiguity,
                        multitagResult.fiducialIDsUsed.size(),
                        totalTagDistance / result.targets.size(),
                        PoseObservationType.PHOTONVISION
                    )
                ); // Observation type
            } else if (!result.targets.isEmpty()) { // Single tag result
                PhotonTrackedTarget target = result.targets.get(0);

                // Calculate robot pose
                Optional<Pose3d> tagPose = aprilTagLayout.getTagPose(target.fiducialId);

                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(
                        tagPose.get().getTranslation(),
                        tagPose.get().getRotation()
                    );
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    // Add tag ID
                    tagIds.add((short) target.fiducialId);

                    // Add observation
                    poseObservations.add(
                        new PoseObservation(
                            result.getTimestampSeconds(),
                            robotPose,
                            target.poseAmbiguity,
                            1,
                            cameraToTarget.getTranslation().getNorm(),
                            PoseObservationType.PHOTONVISION
                        )
                    );
                }
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = poseObservations.toArray(new PoseObservation[poseObservations.size()]);
        poseObservations.clear();

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
        tagIds.clear();
    }
}
