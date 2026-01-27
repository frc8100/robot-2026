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

package frc.util;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

/**
 * Utility methods for working with Spark motor controllers.
 */
public class SparkUtil {

    private SparkUtil() {}

    private static final String ERROR_DASHBOARD_KEY = "SparkUtil/UnreadErrorsList";

    /**
     * Stores whether any error was has been detected by other utility methods.
     */
    private static boolean sparkStickyFault = false;

    /**
     * Stores a list of unread REVLibErrors.
     * Cleared in {@link #periodic()} after being logged.
     */
    private static final List<REVLibError> unreadErrors = new ArrayList<>();

    /**
     * Adds an error to the unread errors list and sets the sticky fault.
     * @param error - The REVLibError to add.
     */
    public static void addError(REVLibError error) {
        unreadErrors.add(error);
        sparkStickyFault = true;
    }

    /**
     * Clears the sticky fault.
     */
    public static void clearStickyFault() {
        sparkStickyFault = false;
    }

    /**
     * @return Whether a sticky fault has occurred.
     */
    public static boolean hasStickyFault() {
        return sparkStickyFault;
    }

    /**
     * Processes and returns a value from a Spark only if the value is valid.
     * @return The value from the supplier if no error occurred, otherwise the value from the elseSupplier.
     */
    public static <T> T ifOkElse(SparkBase spark, Supplier<T> supplier, Supplier<T> elseSupplier) {
        T value = supplier.get();

        REVLibError error = spark.getLastError();

        if (error == REVLibError.kOk) {
            return value;
        } else {
            addError(error);
            return elseSupplier.get();
        }
    }

    /**
     * Processes and returns a value from a Spark only if the value is valid.
     * @return The value from the supplier if no error occurred, otherwise the elseValue.
     */
    public static <T> T ifOkElseValue(SparkBase spark, Supplier<T> supplier, T elseValue) {
        T value = supplier.get();

        REVLibError error = spark.getLastError();

        if (error == REVLibError.kOk) {
            return value;
        } else {
            addError(error);
            // return elseValue;
            // TODO: temporary fix
            return value;
        }
    }

    public static double ifOkOtherwiseZero(SparkBase spark, DoubleSupplier supplier) {
        return ifOkElseValue(spark, supplier::getAsDouble, 0.0);
    }

    /** Processes a value from a Spark only if the value is valid. */
    // public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    //     double value = supplier.getAsDouble();
    //     if (spark.getLastError() == REVLibError.kOk) {
    //         consumer.accept(value);
    //     } else {
    //         sparkStickyFault = true;
    //     }
    // }

    /** Processes a value from a Spark only if the value is valid. */
    // public static void ifOk(SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
    //     double[] values = new double[suppliers.length];
    //     for (int i = 0; i < suppliers.length; i++) {
    //         values[i] = suppliers[i].getAsDouble();
    //         if (spark.getLastError() != REVLibError.kOk) {
    //             sparkStickyFault = true;
    //             return;
    //         }
    //     }
    //     consumer.accept(values);
    // }

    /**
     * Attempts to run the command until no error is produced.
     */
    public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
        for (int i = 0; i < maxAttempts; i++) {
            REVLibError error = command.get();
            if (error == REVLibError.kOk) {
                break;
            } else {
                addError(error);
            }
        }
    }

    /**
     * Attempts to configure a spark max with the default number of max attempts.
     */
    public static void configure(SparkBase spark, SparkBaseConfig config) {
        tryUntilOk(spark, 5, () ->
            spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        );
    }

    public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] = Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
        }

        return odometryTimeStamps;
    }

    public static void warmupErrorLogging() {
        Logger.recordOutput(ERROR_DASHBOARD_KEY, new REVLibError[] {});
    }

    /**
     * Processes sticky faults by logging them and clearing the unread errors list.
     */
    public static void periodic() {
        if (sparkStickyFault) {
            Logger.recordOutput(ERROR_DASHBOARD_KEY, unreadErrors.toArray(new REVLibError[unreadErrors.size()]));
            unreadErrors.clear();
        }
    }
}
