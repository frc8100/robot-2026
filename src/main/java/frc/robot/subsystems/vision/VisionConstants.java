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
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.SimCameraProperties;

/**
 * Vision constants. Some must be configured in the web UI.
 */
public class VisionConstants {

    private VisionConstants() {}

    /**
     * The AprilTag field layout. Note: see TU 12
     */
    // public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(
    //     AprilTagFields.k2025ReefscapeWelded
    // );
    public static AprilTagFieldLayout aprilTagLayout;

    // TODO: Use official field layout when available
    static {
        try {
            aprilTagLayout = new AprilTagFieldLayout(
                Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", "2026-rebuilt-welded.json")
            );
        } catch (Exception e) {
            e.printStackTrace();
            aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        }
    }

    /**
     * A type of game piece observation that can be detected by the neural detector.
     */
    public enum GamePieceObservationType {
        FUEL("Fuel", 1, new TargetModel(Centimeters.of(15).in(Meters)), Centimeters.of(7.5));

        /**
         * @param classID - The class ID from the neural detector.
         * @return The GamePieceObservationType for the given class ID (starting at 1), or null if not found.
         */
        public static GamePieceObservationType fromClassID(int classID) {
            return FUEL;
        }

        /**
         * Gets the GamePieceObservationType from the array index used for inputs.
         * Corresponds with {@link #getArrayIndexForInputs()}.
         * @param index - The array index for inputs, starting at 0.
         * @return The GamePieceObservationType for the given array index.
         */
        public static GamePieceObservationType fromArrayIndex(int index) {
            return FUEL;
        }

        /**
         * @param className - The class name from the neural detector.
         * @return The GamePieceObservationType for the given class name, or null if not found.
         */
        public static GamePieceObservationType fromClassName(String className) {
            return FUEL;
        }

        /**
         * The class name used in the neural detector.
         * Should match the names used in MapleSim.
         * {@link org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo#type}
         */
        public final String className;

        /**
         * The class ID used in the neural detector.
         */
        public final int classID;

        /**
         * The target model for the object.
         * Used for PhotonVision simulation.
         */
        public final TargetModel targetModel;

        /**
         * The height of the target off the floor.
         */
        public final Distance heightOffFloor;

        /**
         * @return The array index for inputs, starting at 0.
         */
        public int getArrayIndexForInputs() {
            return classID - 1;
        }

        private GamePieceObservationType(
            String className,
            int classID,
            TargetModel targetModel,
            Distance heightOffFloor
        ) {
            this.className = className;
            this.classID = classID;
            this.targetModel = targetModel;
            this.heightOffFloor = heightOffFloor;
        }
    }

    /**
     * Camera names, must match names configured on coprocessor
     */
    public static final String CAMERA_0_NAME = "limelight";

    /**
     * Robot to camera 0 transform
     * (Not used by Limelight, configure in web UI instead)
     */
    public static final Transform3d TRANSFORM_TO_CAMERA_0 = new Transform3d(
        Inches.of(9.5), // (28 - 4.5) - 14 or 0.2413 m
        Inches.of(-5.25), // -((28 - 8.75) - 14) or 0.1397 m
        Inches.of(11.25), // 0.28575 m
        Rotation3d.kZero
    );

    public enum CameraPipelines {
        /**
         * AprilTag detection pipeline.
         */
        APRILTAG(0),

        /**
         * Game piece detection pipeline.
         */
        DETECTION(1);

        /**
         * @return The default pipeline to use. Defaults to {@link #APRILTAG}.
         */
        public static CameraPipelines getDefault() {
            return APRILTAG;
        }

        /**
         * The index corresponding to the pipeline in the camera.
         * Should match the index configured in the web UI.
         */
        public final int index;

        private CameraPipelines(int index) {
            this.index = index;
        }
    }

    /**
     * Camera 0 simulated properties
     */
    public static final SimCameraProperties CAMERA_0_PROPERTIES = SimCameraProperties.LL2_1280_720().setFPS(20);

    // Basic filtering thresholds
    public static final double MAX_AMBIGUITY = 0.3;
    public static final double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags
    public static final double LINEAR_STD_DEV_BASELINE = 0.04; // Meters
    public static final double ANGULAR_STD_DEV_BASELINE = 0.12; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static final double[] CAMERA_STD_DEV_FACTORS = new double[] { 1.0 };

    // Multipliers to apply for MegaTag 2 observations
    public static final double LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
    public static final double ANGULAR_STD_DEV_MEGATAG2_FACTOR = Double.POSITIVE_INFINITY; // No rotation data available
}
