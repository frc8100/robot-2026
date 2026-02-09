package frc.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * An extension of InterpolatingDoubleTreeMap that maintains an inverse map for reverse lookups.
 */
public class InvertibleInterpolatingDoubleTreeMap extends InterpolatingDoubleTreeMap {

    /**
     * The inverse map for reverse lookups.
     */
    private final InterpolatingDoubleTreeMap inverseMap = new InterpolatingDoubleTreeMap();

    @Override
    public void put(Double key, Double value) {
        super.put(key, value);
        inverseMap.put(value, key);
    }

    /**
     * @return The inverse map for reverse lookups.
     */
    public InterpolatingDoubleTreeMap getInverseMap() {
        return inverseMap;
    }
}
