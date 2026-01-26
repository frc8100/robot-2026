package frc.robot.subsystems.vision.objectdetection;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;
import frc.util.StdDevSmoother;

public class TrackedVisionTarget {

    /**
     * Logged info for a tracked vision target.
     */
    public record TrackedVisionTargetLoggedInfo(
        double creationTimeSeconds,
        GamePieceObservationType type,
        Translation2d latestPose,
        Translation2d estimatedPose,
        int hits,
        int misses
    ) {}

    /**
     * The type of game piece the target represents.
     */
    public final GamePieceObservationType type;

    /**
     * The time the target was created.
     */
    private final double creationTimeSeconds;

    /**
     * Whether the target has been hit this frame.
     */
    public boolean hasBeenHitThisFrame = false;

    /**
     * Whether the target has been assigned this frame.
     */
    public boolean hasBeenAssignedThisFrame = false;

    /**
     * The number of hits for this target.
     */
    private int hits = 0;

    /**
     * The number of misses for this target.
     */
    public int misses = 0;

    /**
     * The latest pose of the target.
     */
    private Translation2d latestPose;

    /**
     * A smoother for the vision target pose.
     */
    public StdDevSmoother smoother = new StdDevSmoother(2, new double[] { 1.0, 1.0 }, 0.1);

    /**
     * Constructs a new TrackedVisionTarget.
     * @param type - The type of game piece the target represents.
     * @param creationTimeSeconds - The time the target was created.
     * @param initialPose - The initial pose of the target.
     */
    public TrackedVisionTarget(GamePieceObservationType type, double creationTimeSeconds, Translation2d initialPose) {
        this.type = type;
        this.creationTimeSeconds = creationTimeSeconds;
        this.latestPose = initialPose;

        // Set initial state of the Kalman filter
        smoother.overrideState(new double[] { initialPose.getX(), initialPose.getY() });
    }

    /**
     * Updates the pose of the target.
     * @param newPose - The new pose of the target.
     */
    public void addMeasurement(Translation2d newPose) {
        this.latestPose = newPose;
        hits++;

        // Reset misses on a successful update
        misses = 0;
        hasBeenHitThisFrame = true;

        // Update the Kalman filter with the new measurement
        smoother.addMeasurement(new double[] { newPose.getX(), newPose.getY() });
    }

    /**
     * Predicts the next state of the target.
     * Should be called once per update cycle.
     */
    public void update() {
        // kalmanFilter.predict(emptyInput, Constants.LOOP_PERIOD_SECONDS);

        smoother.predict();
    }

    /**
     * @return The estimated pose of the target.
     */
    public Translation2d getEstimatedPose() {
        return new Translation2d(smoother.getCurrentState()[0], smoother.getCurrentState()[1]);
    }

    /**
     * @return The estimated 3D pose of the target.
     */
    public Translation3d getEstimatedPose3d() {
        return new Translation3d(
            smoother.getCurrentState()[0],
            smoother.getCurrentState()[1],
            type.heightOffFloor.in(Meters)
        );
    }

    /**
     * @return True if the target should be deleted due to too many misses.
     */
    public boolean shouldDelete() {
        return misses > 4;
    }

    /**
     * @return The logged info for this tracked vision target.
     */
    public TrackedVisionTargetLoggedInfo getLoggedInfo() {
        return new TrackedVisionTargetLoggedInfo(
            creationTimeSeconds,
            type,
            latestPose,
            getEstimatedPose(),
            hits,
            misses
        );
    }
}
