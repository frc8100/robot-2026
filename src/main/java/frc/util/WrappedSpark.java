package frc.util;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import java.util.Optional;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.simulation.DCMotorSimSupplier;

/**
 * A wrapped Spark motor controller with utility methods.
 */
// TODO: rename
public class WrappedSpark extends SparkWrapper {

    protected final SparkMax motor;
    protected final RelativeEncoder encoder;
    protected final SparkClosedLoopController controller;

    // TODO: Workaround for using trapezoid profile

    public WrappedSpark(SparkMax motor, SmartMotorControllerConfig config) {
        // motor = new SparkMax(canId, MotorType.kBrushless);
        // encoder = motor.getEncoder();
        // controller = motor.getClosedLoopController();

        // motorWrapped = new SparkWrapper(motor, DCMotor.getNEO(1), config);

        super(motor, DCMotor.getNEO(1), config);
        this.motor = motor;
        this.encoder = motor.getEncoder();
        this.controller = motor.getClosedLoopController();
    }

    /**
     * Updates the given data object from this wrapped motor controller.
     * @param dataToUpdate - The data object to update. This is modified in place.
     * @return Whether the data was successfully updated (no sticky faults).
     */
    public boolean updateData(SparkMotorControllerData dataToUpdate) {
        return SubsystemIOUtil.updateDataFromWrappedMotorController(dataToUpdate, this);
    }

    /**
     * Sets up custom simulation for this motor controller. This should be called in the simulation IO class constructor.
     */
    public void setupCustomSimulation() {
        SimulatedBattery.addElectricalAppliances(this::getCustomSupplyCurrent);
    }

    // TODO: this is a bit of a hack, but it allows us to use the existing SparkWrapper closed loop control methods while also allowing us to set the setpoint velocity in simulation (since YAMS does not simulate closed loop control of velocity)
    public void setSetpointVelocity(AngularVelocity velocity) {
        super.setpointVelocity = Optional.ofNullable(velocity);
    }

    /**
     * Gets the supply current of the motor in amps.
     * Adapted from {@link org.ironmaple.simulation.motorsims.MapleMotorSim#getSupplyCurrent}
     * @return The supply current of the motor in amps.
     */
    public Current getCustomSupplyCurrent() {
        return super.getStatorCurrent().times(super.getVoltage().in(Volts) / (RobotController.getBatteryVoltage()));
    }
}
