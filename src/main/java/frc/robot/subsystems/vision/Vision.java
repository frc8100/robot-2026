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

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;
import frc.robot.subsystems.vision.VisionIO.GamePieceObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.objectdetection.GamePiecePoseEstimator;
import frc.util.FieldConstants;
import frc.util.Mutable3x1Vector;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * The vision subsystem. This subsystem processes vision data and sends it to the robot code.
 */
public class Vision extends SubsystemBase {

    /**
     * The consumer for vision data. Usually a method to add a measurement to the pose estimator.
     */
    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
        );
    }

    public enum VisionState {
        /**
         * Before a match starts.
         * Uses a camera to determine initial pose.
         */
        BEFORE_MATCH,

        /**
         * During a match.
         * Sets the pose of the Quest to the pose determined by the camera and use the Quest for pose tracking.
         * Uses a camera for game piece detection.
         */
        DURING_MATCH,
    }

    public final StateMachine<VisionState, Object> stateMachine = new StateMachine<VisionState, Object>(
        VisionState.class,
        "Vision"
    )
        .withDefaultState(new StateMachineState<>(VisionState.BEFORE_MATCH, "BeforeMatch"))
        .withState(new StateMachineState<>(VisionState.DURING_MATCH, "DuringMatch"));

    private final VisionConsumer consumer;

    private final Mutable3x1Vector tempStdDevVector = new Mutable3x1Vector();

    private final List<Pose3d> allTagPoses = new ArrayList<>();
    private final List<Integer> allTagIds = new ArrayList<>();
    private final List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
    private final List<Pose3d> allRobotPosesRejected = new ArrayList<>();

    /**
     * A list of all vision IOs. Each IO corresponds to a camera.
     */
    private final VisionIO[] io;
    private final String[] ioDashboardNames;

    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public final GamePiecePoseEstimator gamePiecePoseEstimator = new GamePiecePoseEstimator();

    private final Swerve swerveSubsystem;
    private final QuestNavSubsystem questNavSubsystem;

    public Vision(Swerve swerveSubsystem, QuestNavSubsystem questSubsystem, VisionIO... io) {
        this.io = io;
        this.swerveSubsystem = swerveSubsystem;
        this.consumer = this.swerveSubsystem::addVisionMeasurement;
        this.questNavSubsystem = questSubsystem;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        this.ioDashboardNames = new String[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
            ioDashboardNames[i] = "Vision/Camera" + Integer.toString(i);
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert(
                "Vision camera " + Integer.toString(i) + " is disconnected.",
                AlertType.kWarning
            );
        }

        // State changes
        stateMachine.onStateChange(VisionState.BEFORE_MATCH, this::setupBeforeMatch);
        stateMachine.onStateChange(VisionState.DURING_MATCH, this::setupDuringMatch);

        // Start in before match state
        setupBeforeMatch();
    }

    private void setupBeforeMatch() {
        // Change to apriltag pipeline
        for (VisionIO visionIO : io) {
            visionIO.setPipeline(VisionConstants.CameraPipelines.APRILTAG);
        }

        // Pause quest nav pose consumption while we determine initial pose
        questNavSubsystem.shouldConsumePoseData = false;
    }

    private void setupDuringMatch() {
        // Change to game piece detection pipeline
        for (VisionIO visionIO : io) {
            visionIO.setPipeline(VisionConstants.CameraPipelines.DETECTION);
        }

        // Reset quest nav pose to vision pose
        questNavSubsystem.setPose(swerveSubsystem.getPose());
        questNavSubsystem.shouldConsumePoseData = true;
    }

    /**
     * Processes the inputs for a single camera by filtering pose observations and sending valid ones to the consumer.
     * @param cameraIndex - The index of the camera.
     * @param inputs - The vision inputs for the camera.
     */
    private void processInputsForCamera(int cameraIndex, VisionIOInputsAutoLogged inputs) {
        // Add tag poses
        for (int tagId : inputs.tagIds) {
            var tagPose = aprilTagLayout.getTagPose(tagId);

            if (tagPose.isPresent()) {
                allTagIds.add(tagId);
                allTagPoses.add(tagPose.get());
            }
        }

        // Loop over pose observations
        for (PoseObservation observation : inputs.poseObservations) {
            // Check whether to reject pose
            boolean rejectPose =
                observation.tagCount() == 0 || // Must have at least one tag
                (observation.tagCount() == 1 && observation.ambiguity() > MAX_AMBIGUITY) || // Cannot be high ambiguity
                Math.abs(observation.pose().getZ()) > MAX_Z_ERROR || // Must have realistic Z coordinate
                // Must be within the field boundaries
                observation.pose().getX() <
                0.0 ||
                observation.pose().getX() > FieldConstants.fieldLength.in(Meters) ||
                observation.pose().getY() < 0.0 ||
                observation.pose().getY() > FieldConstants.fieldWidth.in(Meters);

            // Add pose to log
            if (rejectPose) {
                allRobotPosesRejected.add(observation.pose());
            } else {
                allRobotPosesAccepted.add(observation.pose());
            }

            // Skip if rejected
            if (rejectPose) {
                continue;
            }

            // Calculate standard deviations
            double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
            double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
            double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;
            if (observation.type() == PoseObservationType.MEGATAG_2) {
                linearStdDev *= LINEAR_STD_DEV_MEGATAG2_FACTOR;
                angularStdDev *= ANGULAR_STD_DEV_MEGATAG2_FACTOR;
            }
            if (cameraIndex < CAMERA_STD_DEV_FACTORS.length) {
                linearStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
                angularStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
            }

            tempStdDevVector.set(linearStdDev, linearStdDev, angularStdDev);

            // Send vision observation
            consumer.accept(observation.pose().toPose2d(), observation.timestamp(), tempStdDevVector.getVector());
        }
    }

    @Override
    public void periodic() {
        // Update inputs
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs(ioDashboardNames[i], inputs[i]);
        }

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Process inputs for this camera
            processInputsForCamera(cameraIndex, inputs[cameraIndex]);

            // Update game pieces
            for (int typeIndex = 0; typeIndex < inputs[cameraIndex].gamePieceObservationsByType.length; typeIndex++) {
                GamePieceObservation[] gamePieceObservations =
                    inputs[cameraIndex].gamePieceObservationsByType[typeIndex];

                gamePiecePoseEstimator.updateWithObservations(
                    gamePieceObservations,
                    GamePieceObservationType.fromArrayIndex(typeIndex)
                );
            }
        }

        gamePiecePoseEstimator.processObservations();

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        int[] tagIdsArray = new int[allTagIds.size()];
        for (int i = 0; i < allTagIds.size(); i++) {
            tagIdsArray[i] = allTagIds.get(i);
        }
        Logger.recordOutput("Vision/Summary/TagIds", tagIdsArray);
        Logger.recordOutput(
            "Vision/Summary/RobotPosesAccepted",
            allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()])
        );
        Logger.recordOutput(
            "Vision/Summary/RobotPosesRejected",
            allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()])
        );

        // Clear temporary lists
        allTagPoses.clear();
        allTagIds.clear();
        allRobotPosesAccepted.clear();
        allRobotPosesRejected.clear();

        gamePiecePoseEstimator.logGamePiecePoses();
    }
}
