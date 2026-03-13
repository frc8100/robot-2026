package frc.util;

import edu.wpi.first.wpilibj.RobotBase;
import java.lang.reflect.Field;

/**
 * Utility class for accessing private fields via reflection.
 * ! Should only be used in simulation; using on real robot code might cause stability issues
 * @param <TClass> - The class that contains the field.
 * @param <TFieldValue> - The type of the field value.
 */
// rip encapsulation
public class AccessiblePrivateField<TClass, TFieldValue> {

    /**
     * The reflected field.
     */
    private final Field field;

    /**
     * Creates a new AccessiblePrivateField.
     * @throws RuntimeException if the field is not found or if this is used on real robot code.
     * @param clazz - The class that contains the field.
     * @param fieldName - The name of the field.
     */
    public AccessiblePrivateField(Class<TClass> clazz, String fieldName) {
        // Assert in simulation
        if (RobotBase.isReal()) {
            throw new RuntimeException("AccessiblePrivateField should not be used on real robot code");
        }

        try {
            field = clazz.getDeclaredField(fieldName);
            field.setAccessible(true); // NOSONAR - reflection is necessary for this utility class
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Gets the value of the field from the given object.
     * @throws RuntimeException if the field value cannot be accessed.
     * @param obj - The object to get the field value from.
     * @return The value of the field.
     */
    public TFieldValue get(TClass obj) {
        try {
            @SuppressWarnings("unchecked")
            TFieldValue value = (TFieldValue) field.get(obj);
            return value;
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Sets the value of the field on the given object.
     * @throws RuntimeException if the field value cannot be set.
     * @param obj - The object to set the field value on.
     * @param value - The value to set.
     */
    public void set(TClass obj, TFieldValue value) {
        try {
            field.set(obj, value); // NOSONAR - reflection is necessary for this utility class
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * @return The reflected Field object. Should be used with caution.
     */
    public Field getField() {
        return field;
    }
}
