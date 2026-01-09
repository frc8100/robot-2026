package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

/**
 * Extension of Vision subsystem with simulation support.
 * For object detection sim, there will be a timer separate from PhotonVisionSim's internal timer.
 */
public class VisionSim extends Vision {

    /**
     * A pipeline configuration for a type of object that can be detected by the Neural Detector.
     * @param type - The type and properties of the game piece.
     * @param potentialTargetsSupplier - Supplier for the potential target poses in the field.
     */
    public static record NeuralDetectorSimPipeline(
        GamePieceObservationType type,
        Supplier<List<Pose3d>> potentialTargetsSupplier
    ) {}

    @FunctionalInterface
    public interface GetPosesForGamePieceType {
        /**
         * @return The poses for the given class name.
         * @param className - The class name of the game piece. Corresponds to {@link GamePieceObservationType#className}.
         */
        public List<Pose3d> getPoses(String className);
    }

    /**
     * @return The default neural detector pipelines for all game piece types.
     * @param factory - The factory to get poses for each game piece type. See {@link GetPosesForGamePieceType}.
     */
    public static NeuralDetectorSimPipeline[] getDetectorPipelines(GetPosesForGamePieceType factory) {
        return List.of(GamePieceObservationType.values())
            .stream()
            .map(type -> new NeuralDetectorSimPipeline(type, () -> factory.getPoses(type.className)))
            .toArray(NeuralDetectorSimPipeline[]::new);
    }

    /**
     * Gets a list of VisionTargetSim objects for the given NeuralDetectorSimPipeline.
     * @param pipeline - The NeuralDetectorSimPipeline to get the VisionTargetSim objects for.
     * @return A list of VisionTargetSim objects for the given NeuralDetectorSimPipeline, based on the potential targets from the pipeline's supplier.
     */
    public static List<VisionTargetSim> getVisionTargetSimFromNeuralPipeline(NeuralDetectorSimPipeline pipeline) {
        List<Pose3d> potentialTargets = pipeline.potentialTargetsSupplier.get();

        return potentialTargets.stream().map(pose -> new VisionTargetSim(pose, pipeline.type.targetModel)).toList();
    }

    /**
     * A singleton instance of the vision system simulation.
     */
    private static VisionSystemSim photonVisionSimSystem;

    /**
     * @return The singleton vision system simulation.
     */
    public static VisionSystemSim getVisionSim() {
        // Initialize vision sim if needed
        if (photonVisionSimSystem == null) {
            photonVisionSimSystem = new VisionSystemSim("main");
            photonVisionSimSystem.addAprilTags(VisionConstants.aprilTagLayout);
        }

        return photonVisionSimSystem;
    }

    /**
     * The supplier for the robot pose to use in simulation.
     */
    private final Supplier<Pose2d> robotPoseSupplier;

    /**
     * The neural detector pipelines. Can be null if not used.
     */
    private final NeuralDetectorSimPipeline[] pipelines;

    /**
     * Creates a new VisionSim.
     * @param consumer - The vision consumer for pose measurements.
     * @param robotPoseSupplier - Supplier for the robot pose to use in simulation.
     * @param pipelines - The neural detector pipelines. Set to null to use defaults.
     * @param questNavSubsystem - The QuestNav subsystem.
     * @param ios - The vision IO implementations.
     */
    public VisionSim(
        Swerve swerveSubsystem,
        NeuralDetectorSimPipeline[] pipelines,
        QuestNavSubsystem questNavSubsystem,
        VisionIO... ios
    ) {
        super(swerveSubsystem, questNavSubsystem, ios);
        this.robotPoseSupplier = swerveSubsystem::getActualPose;
        this.pipelines = pipelines;
    }

    /**
     * Updates the vision targets in the simulation based on the current potential targets from each pipeline.
     */
    public void updateVisionTargets() {
        if (pipelines == null) {
            return;
        }

        for (NeuralDetectorSimPipeline pipeline : pipelines) {
            List<Pose3d> potentialTargets = pipeline.potentialTargetsSupplier.get();

            // Remove all existing targets for this pipeline
            getVisionSim().removeVisionTargets(pipeline.type.className);

            // Add new targets
            getVisionSim()
                .addVisionTargets(
                    pipeline.type.className,
                    potentialTargets
                        .stream()
                        .map(pose -> new VisionTargetSim(pose, pipeline.type.targetModel))
                        .toArray(VisionTargetSim[]::new)
                );
        }
    }

    @Override
    public void periodic() {
        // Update vision targets from pipelines
        // updateVisionTargets();

        super.periodic();
    }
}
