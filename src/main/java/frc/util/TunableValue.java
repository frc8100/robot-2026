package frc.util;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableValue implements DoubleSupplier {

    /**
     * A class for a tuning PID controller for a Spark motor controller.
     */
    public static class SparkPIDTunable {

        public final TunableValue kP;
        public final TunableValue kD;

        private final SparkBase motorController;
        private final SparkBaseConfig currentConfig;

        private final String name;

        public SparkPIDTunable(
            String name,
            SparkBase motorController,
            SparkBaseConfig currentConfig,
            double defaultP,
            double defaultD
        ) {
            this.name = name;
            this.motorController = motorController;
            this.currentConfig = currentConfig;

            kP = new TunableValue(name + "/kP", defaultP);
            kD = new TunableValue(name + "/kD", defaultD);

            TunableValue.addRefreshConfigConsumer(this::onRefresh);
        }

        /**
         * Refreshes the PID values from the tunable numbers and updates the motor controller configuration.
         */
        private void onRefresh() {
            // If neither value has changed, do nothing
            if (!hasAnyChanged(kP, kD)) {
                return;
            }

            currentConfig.closedLoop.p(kP.get()).d(kD.get());

            motorController.configure(
                currentConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters
            );

            System.out.println("Updated Spark PID for " + name + ": kP=" + kP.get() + ", kD=" + kD.get());
        }
    }

    /**
     * A functional interface for a consumer that refreshes its config.
     * This is used to update the tunable numbers when the config is changed.
     */
    @FunctionalInterface
    public interface RefreshConfigConsumer {
        void accept();
    }

    /**
     * A list of all the consumers that need to be refreshed when the config is updated.
     * This is used to update the tunable numbers when the config is changed.
     */
    private static final List<RefreshConfigConsumer> refreshConfigConsumers = new ArrayList<>();
    private static final List<TunableValue> tunableValuesWithConsumers = new ArrayList<>();

    /**
     * Adds a consumer to the list of consumers that need to be refreshed when the config is updated.
     * @param consumer The consumer to add
     */
    public static void addRefreshConfigConsumer(RefreshConfigConsumer consumer) {
        refreshConfigConsumers.add(consumer);
    }

    /**
     * Refreshes all the consumers in the list of consumers that need to be refreshed when the config is updated.
     */
    public static Command getRefreshConfigCommand() {
        return new Command() {
            @Override
            public void execute() {
                // Print info
                System.out.println(
                    "Refreshing config for " +
                    refreshConfigConsumers.size() +
                    " consumers and " +
                    tunableValuesWithConsumers.size() +
                    " tunable values with consumers."
                );

                refreshConfigConsumers.forEach(RefreshConfigConsumer::accept);
                tunableValuesWithConsumers.forEach(tunableValue -> tunableValue.onRefresh.accept(tunableValue.get()));
            }

            @Override
            public String getName() {
                return "TunableValue Refresh Config Command";
            }

            @Override
            public boolean isFinished() {
                return true;
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }

    /**
     * The base key for the tuning table in NetworkTables.
     */
    private static final String BASE_TABLE_KEY = "/Tuning";

    /**
     * The key for the tunable number.
     */
    private final String key;

    /**
     * Whether or not the default value has been set.
     */
    private boolean hasDefault = false;

    /**
     * The default value of the number.
     */
    private double defaultValue;

    /**
     * The dashboard number that is used to get the value from the dashboard.
     */
    private LoggedNetworkNumber dashboardNumber;

    /**
     * A map of unique identifiers to the last value that was read for that identifier.
     * This is used to determine if the value has changed since the last time it was read.
     */
    private double lastValue = Double.NaN;

    /**
     * A consumer to run when the value is refreshed.
     */
    public DoubleConsumer onRefresh = (double value) -> {};

    /**
     * Create a new TunableValue
     * @param dashboardKey - Key on dashboard
     */
    public TunableValue(String dashboardKey) {
        this.key = BASE_TABLE_KEY + "/" + dashboardKey;
    }

    /**
     * Create a new TunableValue with the default value
     * @param dashboardKey - Key on dashboard
     * @param defaultValue - Default value
     */
    public TunableValue(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    /**
     * Create a new TunableValue with the default value
     * @param dashboardKey - Key on dashboard
     * @param defaultValue - Default value
     * @param onRefresh - The consumer to run when it is refreshed
     */
    public TunableValue(String dashboardKey, double defaultValue, DoubleConsumer onRefresh) {
        this(dashboardKey, defaultValue);
        this.onRefresh = onRefresh;

        // Add to list of tunable values with consumers
        tunableValuesWithConsumers.add(this);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     * @param defaultValue The default value
     */
    public void initDefault(double defaultValue) {
        // If it already has a default, do nothing
        if (hasDefault) {
            return;
        }

        hasDefault = true;
        this.defaultValue = defaultValue;
        lastValue = defaultValue;

        if (Constants.tuningMode) {
            dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     * @return The current value
     */
    public double get() {
        if (!hasDefault) {
            return 0.0;
        } else {
            return (Constants.tuningMode) ? dashboardNumber.get() : defaultValue;
        }
    }

    /**
     * Checks whether the number has changed since our last check
     * @return True if the number has changed since the last time this method was called, false otherwise.
     */
    public boolean hasChanged() {
        double currentValue = get();

        if (currentValue != lastValue) {
            lastValue = currentValue;
            return true;
        }

        return false;
    }

    /**
     * Checks whether any of the tunableNumbers have changed
     * @param tunableNumbers - All tunable numbers to check
     * @return True if any of the tunable numbers have changed since the last time this method was called, false otherwise.
     */
    public static boolean hasAnyChanged(TunableValue... tunableNumbers) {
        return Arrays.stream(tunableNumbers).anyMatch(TunableValue::hasChanged);
    }

    /**
     * Runs action if any of the tunableNumbers have changed
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple objects. Recommended approach is to pass the result of "hashCode()"
     * @param action Callback to run when any of the tunable numbers have changed. Access tunable numbers in order inputted in method
     * @param tunableNumbers All tunable numbers to check
     */
    public static void ifChanged(Consumer<double[]> action, TunableValue... tunableNumbers) {
        if (hasAnyChanged(tunableNumbers)) {
            action.accept(Arrays.stream(tunableNumbers).mapToDouble(TunableValue::get).toArray());
        }
    }

    /** Runs action if any of the tunableNumbers have changed */
    public static void ifChanged(Runnable action, TunableValue... tunableNumbers) {
        ifChanged(values -> action.run(), tunableNumbers);
    }

    @Override
    public double getAsDouble() {
        return get();
    }
}
