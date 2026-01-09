package frc.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A mutable 3x1 vector class for storing and updating 3D vectors.
 * Avoids unnecessary allocations by reusing the same underlying matrix.
 */
public class Mutable3x1Vector {

    /**
     * The underlying 3x1 vector matrix.
     */
    private final Matrix<N3, N1> vector = VecBuilder.fill(0.0, 0.0, 0.0);

    /**
     * @return The underlying 3x1 vector matrix.
     */
    public Matrix<N3, N1> getVector() {
        return vector;
    }

    /**
     * Sets the values of the vector.
     * @param x - The x value.
     * @param y - The y value.
     * @param z - The z value.
     */
    public void set(double x, double y, double z) {
        vector.set(0, 0, x);
        vector.set(1, 0, y);
        vector.set(2, 0, z);
    }
}
