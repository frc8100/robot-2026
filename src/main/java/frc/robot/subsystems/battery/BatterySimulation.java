package frc.robot.subsystems.battery;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.ironmaple.simulation.motorsims.MapleMotorSim;

/**
 *
 *
 * <h1>Simulates the main battery of the robot.</h1>
 *
 * <p>This class simulates the behavior of a robot's battery. Electrical appliances can be added to the battery to draw
 * current. The battery voltage is affected by the current drawn from various appliances.
 */
public class BatterySimulation {

    // TODO: make this work alongside maplesim SimulatedBattery instead of replacing it

    public record ElectricalAppliance(int pdhPortId, Supplier<Current> currentSupplier, String name) {}

    public static final Current CURRENT_EPSILON = Amps.of(0.01);

    private static BatterySimulation instance = null;

    public static BatterySimulation getInstance() {
        // Check if in simulation
        if (RobotBase.isReal()) {
            DriverStation.reportError(
                "[BatterySimulation] Warning: Attempting to access BatterySimulation in non-simulation environment",
                true
            );
        }

        if (instance == null) {
            instance = new BatterySimulation();
        }

        return instance;
    }

    private final List<ElectricalAppliance> electricalAppliances = new ArrayList<>();

    private final double[] cachedChannelCurrentsAmps = new double[BatteryLogger.NUM_CHANNELS];

    private BatterySimulation() {}

    /**
     *
     *
     * <h2>Adds a custom electrical appliance.</h2>
     *
     * <p>Connects the electrical appliance to the battery, allowing it to draw current from the battery.
     *
     * @param customElectricalAppliances The supplier for the current drawn by the appliance.
     */
    public void addElectricalAppliances(Supplier<Current> customElectricalAppliances) {
        // electricalAppliances.add(customElectricalAppliances);
    }

    /**
     *
     *
     * <h2>Adds a motor to the list of electrical appliances.</h2>
     *
     * <p>The motor will draw current from the battery.
     *
     * @param mapleMotorSim The motor simulation object.
     */
    public void addMotor(MapleMotorSim mapleMotorSim) {
        // electricalAppliances.add(mapleMotorSim::getSupplyCurrent);
    }

    /**
     *
    //  *
    //  * <h2>Obtains the total current drawn from the battery.</h2>
    //  *
    //  * <p>Iterates through all the appliances to obtain the total current used.
    //  *
    //  * @return The total current as a {@link Current} object.
     */
    private Current getTotalCurrentDrawn() {
        double totalCurrentAmps = electricalAppliances
            .stream()
            .mapToDouble(appliance -> {
                double currentAmps = appliance.currentSupplier.get().in(Amps);
                cachedChannelCurrentsAmps[appliance.pdhPortId] = currentAmps;
                return currentAmps;
            })
            .sum();
        return Amps.of(totalCurrentAmps);
    }

    /**
     * @return An array of the current drawn from each channel in amps. The index of the array corresponds to the PDH port number.
     * Note: This returns a reference to an array that will be modified.
     */
    public double[] getCachedChannelCurrentsAmps() {
        return cachedChannelCurrentsAmps;
    }

    public void periodic() {}
}
