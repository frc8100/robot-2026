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

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Unit;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class SparkUtil {

    private SparkUtil() {}

    /** Stores whether any error was has been detected by other utility methods. */
    public static boolean sparkStickyFault = false;

    /**
     * Processes and returns a value from a Spark only if the value is valid.
     * @return The value from the supplier if no error occurred, otherwise the value from the elseSupplier.
     */
    public static <T> T ifOkElse(SparkBase spark, Supplier<T> supplier, Supplier<T> elseSupplier) {
        T value = supplier.get();

        if (spark.getLastError() == REVLibError.kOk) {
            return value;
        } else {
            sparkStickyFault = true;
            return elseSupplier.get();
        }
    }

    /**
     * Processes and returns a value from a Spark only if the value is valid.
     * @return The value from the supplier if no error occurred, otherwise the elseValue.
     */
    public static <T> T ifOkElseValue(SparkBase spark, Supplier<T> supplier, T elseValue) {
        T value = supplier.get();

        if (spark.getLastError() == REVLibError.kOk) {
            return value;
        } else {
            sparkStickyFault = true;
            return elseValue;
        }
    }

    public static double ifOkOtherwiseZero(SparkBase spark, DoubleSupplier supplier) {
        return ifOkElse(spark, supplier::getAsDouble, () -> 0.0);
    }

    // @SuppressWarnings("unchecked")
    // public static <U extends Measure<?>> U ifOkOtherwiseZero(
    //     U baseUnit,
    //     SparkBase spark,
    //     Supplier<U> supplier
    // ) {
    //     return ifOkElse(spark, supplier, () -> (U) baseUnit.);
    // }

    /** Processes a value from a Spark only if the value is valid. */
    public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
        double value = supplier.getAsDouble();
        if (spark.getLastError() == REVLibError.kOk) {
            consumer.accept(value);
        } else {
            sparkStickyFault = true;
        }
    }

    /** Processes a value from a Spark only if the value is valid. */
    public static void ifOk(SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
        double[] values = new double[suppliers.length];
        for (int i = 0; i < suppliers.length; i++) {
            values[i] = suppliers[i].getAsDouble();
            if (spark.getLastError() != REVLibError.kOk) {
                sparkStickyFault = true;
                return;
            }
        }
        consumer.accept(values);
    }

    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error == REVLibError.kOk) {
                break;
            } else {
                sparkStickyFault = true;
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
}
