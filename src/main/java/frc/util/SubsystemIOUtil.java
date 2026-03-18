package frc.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.Constants;
import java.nio.ByteBuffer;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Utility class for subsystem IO.
 */
public final class SubsystemIOUtil {

    private SubsystemIOUtil() {}

    /**
     * The data structure for Spark motor controllers.
     */
    public static class SparkMotorControllerData implements StructSerializable {

        // Default values for faults and warnings to reduce allocation as they are immutable
        public static final SparkBase.Faults DEFAULT_FAULTS = new SparkBase.Faults(0);
        public static final SparkBase.Warnings DEFAULT_WARNINGS = new SparkBase.Warnings(0);

        public static boolean hasFault(SparkBase.Faults faults) {
            return faults.rawBits != 0;
        }

        public static boolean hasWarning(SparkBase.Warnings warnings) {
            return warnings.rawBits != 0;
        }

        private static final int NUMBER_OF_DOUBLE_FIELDS = 6;
        private static final int NUMBER_OF_INT_FIELDS = 2;

        /**
         * The position of the mechanism. Multiplied by a conversion factor from the motor rotations.
         */
        public final MutAngle positionAngle = Radians.mutable(0);

        /**
         * The PID setpoint position of the mechanism.
         */
        public final MutAngle setpointAngle = Radians.mutable(0);

        /**
         * The velocity of the mechanism. Multiplied by a conversion factor from the motor RPM.
         */
        public final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

        /**
         * The applied voltage to the motor.
         * Calculated as (applied output) * (bus voltage).
         */
        public final MutVoltage appliedVolts = Volts.mutable(0);

        /**
         * The torque current applied by the motor.
         */
        public final MutCurrent torqueCurrent = Amps.mutable(0);

        /**
         * The temperature of the motor. Not simulated.
         */
        public final MutTemperature temperature = Celsius.mutable(0);

        /**
         * The faults of the motor controller. Not simulated.
         */
        protected SparkBase.Faults faults = DEFAULT_FAULTS;

        /**
         * The warnings of the motor controller. Not simulated.
         */
        protected SparkBase.Warnings warnings = DEFAULT_WARNINGS;

        /**
         * Creates a new Spark motor controller data object with default values (zeroes).
         */
        public SparkMotorControllerData() {}

        /**
         * Creates a new Spark motor controller data object from the given double values.
         */
        public SparkMotorControllerData(
            double positionAngleRad,
            double setpointAngleRad,
            double velocityRadPerSec,
            double appliedVolts,
            double torqueCurrentAmps,
            double temperatureCelsius
        ) {
            mut_replace(
                positionAngleRad,
                setpointAngleRad,
                velocityRadPerSec,
                appliedVolts,
                torqueCurrentAmps,
                temperatureCelsius
            );
        }

        /**
         * Creates a new Spark motor controller data object from the given Measurement values.
         */
        public SparkMotorControllerData(
            Angle positionAngle,
            Angle setpointAngle,
            AngularVelocity velocity,
            Voltage appliedVolts,
            Current torqueCurrent,
            Temperature temperature
        ) {
            mut_replace(positionAngle, setpointAngle, velocity, appliedVolts, torqueCurrent, temperature);
        }

        public SparkBase.Faults getFaults() {
            return faults;
        }

        public SparkBase.Warnings getWarnings() {
            return warnings;
        }

        /**
         * Replaces all values in this data object with the given double values.
         */
        public void mut_replace(
            double positionAngleRad,
            double setpointAngleRad,
            double velocityRadPerSec,
            double appliedVolts,
            double torqueCurrentAmps,
            double temperatureCelsius
        ) {
            this.positionAngle.mut_replace(positionAngleRad, Radians);
            this.setpointAngle.mut_replace(setpointAngleRad, Radians);
            this.velocity.mut_replace(velocityRadPerSec, RadiansPerSecond);
            this.appliedVolts.mut_replace(appliedVolts, Volts);
            this.torqueCurrent.mut_replace(torqueCurrentAmps, Amps);
            this.temperature.mut_replace(temperatureCelsius, Celsius);
        }

        /**
         * Replaces all values in this data object with the given Measurement values.
         */
        public void mut_replace(
            Angle positionAngle,
            Angle setpointAngle,
            AngularVelocity velocity,
            Voltage appliedVolts,
            Current torqueCurrent,
            Temperature temperature
        ) {
            this.positionAngle.mut_replace(positionAngle);
            this.setpointAngle.mut_replace(setpointAngle);
            this.velocity.mut_replace(velocity);
            this.appliedVolts.mut_replace(appliedVolts);
            this.torqueCurrent.mut_replace(torqueCurrent);
            this.temperature.mut_replace(temperature);
        }

        /**
         * The struct used to serialize and deserialize Spark motor controller data.
         */
        public static final SparkMotorControllerDataStruct struct = new SparkMotorControllerDataStruct();

        /**
         * The struct implementation for Spark motor controller data.
         */
        public static class SparkMotorControllerDataStruct implements Struct<SparkMotorControllerData> {

            @Override
            public Class<SparkMotorControllerData> getTypeClass() {
                return SparkMotorControllerData.class;
            }

            @Override
            public String getTypeName() {
                return "SparkMotorControllerData";
            }

            @Override
            public int getSize() {
                return NUMBER_OF_DOUBLE_FIELDS * kSizeDouble + NUMBER_OF_INT_FIELDS * kSizeInt32;
            }

            @Override
            public String getSchema() {
                return (
                    "double positionAngleRad;" +
                    "double setpointAngleRad;" +
                    "double velocityRadPerSec;" +
                    "double appliedVolts;" +
                    "double torqueCurrentAmps;" +
                    "double temperatureCelsius;" +
                    "int faults;" +
                    "int warnings;"
                );
            }

            @Override
            public SparkMotorControllerData unpack(ByteBuffer bb) {
                SparkMotorControllerData output = new SparkMotorControllerData(
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble()
                );

                output.faults = new SparkBase.Faults(bb.getInt());
                output.warnings = new SparkBase.Warnings(bb.getInt());
                return output;
            }

            @Override
            public void pack(ByteBuffer bb, SparkMotorControllerData data) {
                bb.putDouble(data.positionAngle.in(Radians));
                bb.putDouble(data.setpointAngle.in(Radians));
                bb.putDouble(data.velocity.in(RadiansPerSecond));
                bb.putDouble(data.appliedVolts.in(Volts));
                bb.putDouble(data.torqueCurrent.in(Amps));
                bb.putDouble(data.temperature.in(Celsius));
                bb.putInt(data.faults.rawBits);
                bb.putInt(data.warnings.rawBits);
            }
        }
    }

    /**
     * Updates the data from a Spark motor controller wrapped in a SparkWrapper.
     * @param dataToUpdate - The data to update.
     * @param motorController - The motor controller to get data from.
     * @return Whether the data was successfully updated (no sticky faults).
     */
    public static boolean updateDataFromWrappedMotorController(
        SparkMotorControllerData dataToUpdate,
        SparkWrapper motorController
    ) {
        SparkBase spark = (SparkBase) motorController.getMotorController();

        // Reset spark sticky fault
        SparkUtil.clearStickyFault();

        dataToUpdate.positionAngle.mut_replace(
            SparkUtil.ifOkElseValue(spark, motorController::getMechanismPosition, Radians.zero())
        );
        dataToUpdate.setpointAngle.mut_replace(
            SparkUtil.ifOkElseValue(
                spark,
                () -> motorController.getMechanismPositionSetpoint().orElse(Radians.zero()),
                Radians.zero()
            )
        );
        dataToUpdate.velocity.mut_replace(
            SparkUtil.ifOkElseValue(spark, motorController::getMechanismVelocity, RadiansPerSecond.zero())
        );
        dataToUpdate.appliedVolts.mut_replace(
            // SparkUtil.ifOkElseValue(spark, motorController::getVoltage, Volts.zero())
            SparkUtil.ifOkOtherwiseZero(spark, () -> spark.getAppliedOutput() * spark.getBusVoltage()),
            Volts
        );
        dataToUpdate.torqueCurrent.mut_replace(
            // SparkUtil.ifOkElseValue(spark, motorController::getStatorCurrent, Amps.zero())
            SparkUtil.ifOkOtherwiseZero(spark, spark::getOutputCurrent),
            Amps
        );
        dataToUpdate.temperature.mut_replace(
            SparkUtil.ifOkElseValue(spark, motorController::getTemperature, Celsius.zero())
        );

        dataToUpdate.faults = SparkUtil.ifOkElseValue(spark, spark::getFaults, SparkMotorControllerData.DEFAULT_FAULTS);
        dataToUpdate.warnings = SparkUtil.ifOkElseValue(
            spark,
            spark::getWarnings,
            SparkMotorControllerData.DEFAULT_WARNINGS
        );

        return !SparkUtil.hasStickyFault();
    }

    public static boolean updateDataFromWrappedSparkOrSimulation(
        SparkMotorControllerData dataToUpdate,
        SparkWrapper wrappedMotorController,
        SparkBase motorController,
        RelativeEncoder relativeEncoder,
        SparkClosedLoopController closedLoopController
    ) {
        // If in simulation, use the wrapped motor controller method
        if (Constants.currentMode == Constants.Mode.SIM) {
            // Note: shouldn't have any issues with replay because this method should not be called in replay (empty IO implementation used)
            return updateDataFromWrappedMotorController(dataToUpdate, wrappedMotorController);
        } else {
            // Otherwise, use the standard Spark method
            return updateDataFromSpark(dataToUpdate, motorController, relativeEncoder, closedLoopController);
        }
    }

    public static boolean updateDataFromSpark(
        SparkMotorControllerData dataToUpdate,
        SparkBase motorController,
        RelativeEncoder relativeEncoder
    ) {
        // Reset spark sticky fault
        SparkUtil.clearStickyFault();

        dataToUpdate.positionAngle.mut_replace(
            SparkUtil.ifOkOtherwiseZero(motorController, relativeEncoder::getPosition),
            Radians
        );
        dataToUpdate.velocity.mut_replace(
            SparkUtil.ifOkOtherwiseZero(motorController, relativeEncoder::getVelocity),
            RadiansPerSecond
        );
        dataToUpdate.appliedVolts.mut_replace(
            SparkUtil.ifOkOtherwiseZero(
                motorController,
                () -> motorController.getAppliedOutput() * motorController.getBusVoltage()
            ),
            Volts
        );
        dataToUpdate.torqueCurrent.mut_replace(
            SparkUtil.ifOkOtherwiseZero(motorController, motorController::getOutputCurrent),
            Amps
        );
        dataToUpdate.temperature.mut_replace(
            SparkUtil.ifOkOtherwiseZero(motorController, motorController::getMotorTemperature),
            Celsius
        );

        dataToUpdate.faults = SparkUtil.ifOkElseValue(
            motorController,
            motorController::getFaults,
            SparkMotorControllerData.DEFAULT_FAULTS
        );
        dataToUpdate.warnings = SparkUtil.ifOkElseValue(
            motorController,
            motorController::getWarnings,
            SparkMotorControllerData.DEFAULT_WARNINGS
        );

        return !SparkUtil.hasStickyFault();
    }

    /**
     * Updates the data from a Spark motor controller and its relative encoder.
     * @param dataToUpdate - The data to update.
     * @param motorController - The motor controller to get data from.
     * @param relativeEncoder - The relative encoder to get position and velocity from.
     * @return Whether the data was successfully updated (no sticky faults).
     */
    public static boolean updateDataFromSpark(
        SparkMotorControllerData dataToUpdate,
        SparkBase motorController,
        RelativeEncoder relativeEncoder,
        SparkClosedLoopController closedLoopController
    ) {
        // Reset spark sticky fault
        SparkUtil.clearStickyFault();

        dataToUpdate.positionAngle.mut_replace(
            SparkUtil.ifOkOtherwiseZero(motorController, relativeEncoder::getPosition),
            Radians
        );
        dataToUpdate.setpointAngle.mut_replace(
            SparkUtil.ifOkOtherwiseZero(motorController, closedLoopController::getSetpoint),
            Radians
        );
        dataToUpdate.velocity.mut_replace(
            SparkUtil.ifOkOtherwiseZero(motorController, relativeEncoder::getVelocity),
            RadiansPerSecond
        );
        dataToUpdate.appliedVolts.mut_replace(
            SparkUtil.ifOkOtherwiseZero(
                motorController,
                () -> motorController.getAppliedOutput() * motorController.getBusVoltage()
            ),
            Volts
        );
        dataToUpdate.torqueCurrent.mut_replace(
            SparkUtil.ifOkOtherwiseZero(motorController, motorController::getOutputCurrent),
            Amps
        );
        dataToUpdate.temperature.mut_replace(
            SparkUtil.ifOkOtherwiseZero(motorController, motorController::getMotorTemperature),
            Celsius
        );

        dataToUpdate.faults = SparkUtil.ifOkElseValue(
            motorController,
            motorController::getFaults,
            SparkMotorControllerData.DEFAULT_FAULTS
        );
        dataToUpdate.warnings = SparkUtil.ifOkElseValue(
            motorController,
            motorController::getWarnings,
            SparkMotorControllerData.DEFAULT_WARNINGS
        );

        return !SparkUtil.hasStickyFault();
    }
}
