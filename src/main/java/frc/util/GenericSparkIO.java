package frc.util;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Generic interface for Spark IO. Should be extended by IO implementation for involving 1 spark motor.
 * @param <TInputs> The type of the inputs for the IO.
 * See {@link GenericSparkIOInputs} for an example implementation of IO inputs.
 */
public interface GenericSparkIO<TInputs> {
    /**
     * The inputs that should be supplied by the IO interface.
     * Contains fields for the motor position, velocity, voltage, current, and temperature.
     * Can be extended to include additional fields, and should include a @AutoLog annotation.
     */
    public static class GenericSparkIOInputs {

        /**
         * Whether the motor is connected.
         */
        public boolean connected = true;

        /**
         * The position of the motor in radians. When turning counterclockwise, this value increases.
         */
        public double positionRad = 0.0;

        /**
         * The velocity of the motor in radians per second. When turning counterclockwise, this value increases.
         */
        public double velocityRadPerSec = 0.0;

        /**
         * The voltage applied to the motor.
         */
        public double appliedVolts = 0.0;

        /**
         * The supply current of the motor in amps.
         */
        public double supplyCurrentAmps = 0.0;

        /**
         * The torque current of the motor in amps.
         */
        public double torqueCurrentAmps = 0.0;
        /**
         * The temperature of the motor in celsius.
         */
        // public double tempCelsius = 0.0;
    }

    /**
     * The configuration for the Spark max motor.
     * Used as a parameter for the {@link #getDefaultConfig()} method.
     */
    public static class GenericSparkIOConfig {

        /**
         * The behavior of the motor when idle.
         */
        public SparkBaseConfig.IdleMode idleMode = SparkBaseConfig.IdleMode.kBrake;

        /**
         * Whether the motor is inverted.
         */
        public boolean inverted = false;

        /**
         * The current limit of the motor, in amps.
         */
        public int smartCurrentLimit = 40;

        /**
         * Converts from position in rotations to the desired unit.
         */
        public double positionConversionFactor = 1.0;

        /**
         * Creates a new configuration with default values.
         * Should be overridden by the user.
         */
        public GenericSparkIOConfig() {}

        /**
         * Gets the configuration for the Spark max motor.
         */
        public SparkMaxConfig getConfig() {
            return getDefaultConfig(this);
        }
    }

    /**
     * Note: use {@link GenericSparkIOConfig#getConfig()} instead.
     * Gets the default configuration for the Spark max motor.
     * @param config - The configuration to use. See {@link GenericSparkIOConfig}.
     * @return The default configuration for the Spark max motor.
     */
    @Deprecated
    private static SparkMaxConfig getDefaultConfig(GenericSparkIOConfig config) {
        SparkMaxConfig outputConfig = new SparkMaxConfig();

        outputConfig.idleMode(config.idleMode).inverted(config.inverted).smartCurrentLimit(config.smartCurrentLimit);

        // Configure the encoder
        outputConfig.encoder
            .positionConversionFactor(config.positionConversionFactor)
            .velocityConversionFactor(config.positionConversionFactor / 60)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        // Configure the signals
        outputConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        return outputConfig;
    }

    /**
     * Updates the inputs for the IO. Should be called periodically, and should mutate the inputs.
     * @param inputs - The inputs to update.
     */
    public default void updateInputs(TInputs inputs) {}
}
