package frc.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Unit;
import org.littletonrobotics.junction.LogTable;

/**
 * Utility class for updating mutable units from a log table.
 */
public class MutableUnitsFromLog {

    private MutableUnitsFromLog() {}

    /**
     * Updates a mutable measure from a log table.
     * @param <U> - Unit type
     * @param <Base> - Base measure type
     * @param <M> - Mutable measure type
     * @param table - Log table to get data from
     * @param key - Key to get data from
     * @param measure - Mutable measure to update
     */
    public static <
        U extends Unit, Base extends U, M extends MutableMeasure<U, Base, M>
    > void updateMutableMeasureFromLog(LogTable table, String key, M measure) {
        // Get the base value from the log table
        double baseValue = table.get(key).getDouble(measure.baseUnitMagnitude());

        // Update the mutable measure
        measure.mut_replace(baseValue, measure.baseUnit());
    }
}
