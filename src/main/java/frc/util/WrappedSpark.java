package frc.util;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * A wrapped Spark motor controller with utility methods.
 */
// TODO: rename
public class WrappedSpark extends SparkWrapper {

    // protected final SparkMax motor;
    // protected final RelativeEncoder encoder;
    // protected final SparkClosedLoopController controller;

    // protected final SparkWrapper motorWrapped;

    public WrappedSpark(SparkMax motor, SmartMotorControllerConfig config) {
        // motor = new SparkMax(canId, MotorType.kBrushless);
        // encoder = motor.getEncoder();
        // controller = motor.getClosedLoopController();

        // motorWrapped = new SparkWrapper(motor, DCMotor.getNEO(1), config);

        super(motor, DCMotor.getNEO(1), config);
    }

    /**
     * Updates the given data object from this wrapped motor controller.
     * @param dataToUpdate - The data object to update. This is modified in place.
     * @return Whether the data was successfully updated (no sticky faults).
     */
    public boolean updateData(SparkMotorControllerData dataToUpdate) {
        return SubsystemIOUtil.updateDataFromWrappedMotorController(dataToUpdate, this);
    }
}
