package frc.util;

/**
 * A filter somewhat similar to a Kalman filter but does not have a state prediction step.
 * Smooths noisy measurements using standard deviation to weight them.
 */
public class StdDevSmoother {

    private final int stateRows;

    /**
     * The current state vector.
     */
    private final double[] currentState;

    /**
     * The current standard deviations of the state vector.
     */
    private final double[] currentStateStdDevs;

    private final double[] currentMeasurementStdDevs;

    private final double processStdDev;

    private boolean hasState = false;

    /**
     * Creates a new StdDevSmoother.
     * @param stateRows - The number of rows in the state vector.
     * @param initialStdDevs - The initial standard deviations for each state element and measurement.
     * @param processStdDev - The standard deviation of the process noise injected during prediction.
     */
    public StdDevSmoother(int stateRows, double[] initialStdDevs, double processStdDev) {
        this.stateRows = stateRows;
        this.currentState = new double[stateRows];
        this.currentStateStdDevs = new double[stateRows];
        System.arraycopy(initialStdDevs, 0, this.currentStateStdDevs, 0, stateRows);

        currentMeasurementStdDevs = new double[stateRows];
        System.arraycopy(initialStdDevs, 0, this.currentMeasurementStdDevs, 0, stateRows);

        this.processStdDev = processStdDev;
    }

    /**
     * Overrides the current state with a new state.
     * @param newState - The new state vector.
     */
    public void overrideState(double[] newState) {
        System.arraycopy(newState, 0, this.currentState, 0, stateRows);
        hasState = true;
    }

    public void overrideStateStdDevs(double[] newStdDevs) {
        System.arraycopy(newStdDevs, 0, this.currentStateStdDevs, 0, stateRows);
    }

    /**
     * Sets the current measurement standard deviations.
     * @param measurementStdDevs - The standard deviations of the measurement.
     */
    public void setMeasurementStdDevs(double[] measurementStdDevs) {
        System.arraycopy(measurementStdDevs, 0, this.currentMeasurementStdDevs, 0, stateRows);
    }

    /**
     * Adds a new measurement to the smoother.
     * @param measurement - The measurement vector.
     * @param measurementStdDevs - The standard deviations of the measurement.
     */
    public void addMeasurement(double[] measurement) {
        // If no yet, initialize it with the first measurement
        if (!hasState) {
            overrideState(measurement);
            return;
        }

        // Update each state element using the Kalman gain formula
        for (int i = 0; i < stateRows; i++) {
            double priorVariance = currentStateStdDevs[i] * currentStateStdDevs[i];
            double measurementVariance = currentMeasurementStdDevs[i] * currentMeasurementStdDevs[i];

            double kalmanGain = priorVariance / (priorVariance + measurementVariance);

            currentState[i] = currentState[i] + kalmanGain * (measurement[i] - currentState[i]);

            currentStateStdDevs[i] = Math.sqrt((1 - kalmanGain) * priorVariance);
        }
    }

    public void addMeasurement(double[] measurement, double[] measurementStdDevs) {
        setMeasurementStdDevs(measurementStdDevs);
        addMeasurement(measurement);
    }

    /**
     * Predicts the next state by injecting process noise.
     */
    public void predict() {
        // Inject some process noise to avoid overconfidence/slow convergence
        for (int i = 0; i < stateRows; i++) {
            double priorVariance = currentStateStdDevs[i] * currentStateStdDevs[i];
            double processVariance = processStdDev * processStdDev;
            priorVariance += processVariance;
            currentStateStdDevs[i] = Math.sqrt(priorVariance);
        }
    }

    /**
     * Gets the current smoothed state.
     * @return The smoothed state vector.
     */
    public double[] getCurrentState() {
        return currentState;
    }
}
