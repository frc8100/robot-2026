package frc.robot.subsystems.vision.objectdetection;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N0;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.LinearSystem;
import frc.robot.subsystems.vision.VisionConstants.GamePieceObservationType;
import frc.util.StdDevSmoother;

public class TrackedVisionTarget {

    /**
     * Logged info for a tracked vision target.
     */
    public static record TrackedVisionTargetLoggedInfo(
        double creationTimeSeconds,
        GamePieceObservationType type,
        Pose2d latestPose,
        Pose2d estimatedPose,
        int hits,
        int misses
    ) {}

    // State and measurement standard deviations
    // TODO: Move to constants file
    public static final Matrix<N4, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 1.0, 1.0);
    public static final Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(0.1, 0.1);

    private static final Matrix<N0, N1> emptyInput = new Matrix<>(N0.instance, N1.instance);

    /**
     * A constant-velocity LinearSystem for a 4-state ([x,y,vx,vy]) and 2-output ([x,y]) measurement.
     * The returned system has input dimension 0.
     */
    public static final LinearSystem<N4, N0, N2> visionTargetLinearSystem = new LinearSystem<>(
        // A matrix (4x4) - Continuous Time
        // Row 1: dx/dt = vx
        // Row 2: dy/dt = vy
        // Row 3: dvx/dt = 0 (Constant velocity assumption)
        // Row 4: dvy/dt = 0
        new Matrix<>(
            N4.instance,
            N4.instance,
            // @formatter:off
            new double[] {
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0
            }
            // @formatter:on
        ),
        // B is 4x0 (no inputs)
        new Matrix<>(N4.instance, N0.instance),
        // C maps state -> measurement (2x4)
        // Measure x and y directly.
        new Matrix<>(
            N2.instance,
            N4.instance,
            // @formatter:off
            new double[] {
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0
            }
            // @formatter:on
        ),
        // D is 2x0 (no feedthrough)
        new Matrix<>(N2.instance, N0.instance)
    );

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
    private Pose2d latestPose;

    /**
     * A Kalman filter for estimating the pose of the vision target.
     * The state vector is [x, y, vx, vy].
     * The output vector is [x, y].
     */
    // public KalmanFilter<N4, N0, N2> kalmanFilter = new KalmanFilter<>(
    //     N4.instance,
    //     N2.instance,
    //     visionTargetLinearSystem,
    //     stateStdDevs,
    //     measurementStdDevs,
    //     Constants.LOOP_PERIOD_SECONDS
    // );

    public StdDevSmoother smoother = new StdDevSmoother(3, new double[] { 1.0, 1.0, 1.0 }, 0.1);

    /**
     * Constructs a new TrackedVisionTarget.
     * @param type - The type of game piece the target represents.
     * @param creationTimeSeconds - The time the target was created.
     * @param initialPose - The initial pose of the target.
     */
    public TrackedVisionTarget(GamePieceObservationType type, double creationTimeSeconds, Pose2d initialPose) {
        this.type = type;
        this.creationTimeSeconds = creationTimeSeconds;
        this.latestPose = initialPose;

        // Seed filter
        // Set x and y to the measurement
        // Leave vx and vy at 0
        // kalmanFilter.setXhat(0, initialPose.getX());
        // kalmanFilter.setXhat(1, initialPose.getY());

        smoother.overrideState(
            new double[] { initialPose.getX(), initialPose.getY(), initialPose.getRotation().getRadians() }
        );
    }

    // private final Matrix<N2, N1> cachedMeasurement = new Matrix<>(N2.instance, N1.instance);

    /**
     * Updates the pose of the target.
     * @param newPose - The new pose of the target.
     */
    public void addMeasurement(Pose2d newPose) {
        this.latestPose = newPose;
        hits++;

        // Reset misses on a successful update
        misses = 0;
        hasBeenHitThisFrame = true;

        // Update the Kalman filter with the new measurement
        // cachedMeasurement.set(0, 0, newPose.getX());
        // cachedMeasurement.set(1, 0, newPose.getY());
        // kalmanFilter.correct(emptyInput, cachedMeasurement);

        smoother.addMeasurement(new double[] { newPose.getX(), newPose.getY(), newPose.getRotation().getRadians() });
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
    public Pose2d getEstimatedPose() {
        return new Pose2d(
            smoother.getCurrentState()[0],
            smoother.getCurrentState()[1],
            new Rotation2d(smoother.getCurrentState()[2])
        );
    }

    /**
     * @return The estimated 3D pose of the target.
     */
    public Pose3d getEstimatedPose3d() {
        return new Pose3d(
            // kalmanFilter.getXhat(0),
            // kalmanFilter.getXhat(1),
            smoother.getCurrentState()[0],
            smoother.getCurrentState()[1],
            type.heightOffFloor.in(Meters),
            new Rotation3d(0, 0, smoother.getCurrentState()[2])
        );
    }

    /**
     * @return True if the target should be deleted due to too many misses.
     */
    public boolean shouldDelete() {
        return misses > 5;
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
