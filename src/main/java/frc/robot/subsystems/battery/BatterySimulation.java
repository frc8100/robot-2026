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

    // Nominal voltage for a fully charged battery
    private static final Voltage BATTERY_NOMINAL_VOLTAGE = Volts.of(12.5);

    // Filter to smooth the current readings.
    private final LinearFilter currentFilter = LinearFilter.movingAverage(50);

    private final ElectricalAppliance[] electricalAppliances;

    // The current battery voltage in volts.
    private Voltage batteryVoltageVolts = BATTERY_NOMINAL_VOLTAGE;

    private BatterySimulation() {
        this.electricalAppliances = new ElectricalAppliance[BatteryLogger.NUM_CHANNELS];

        for (int i = 0; i < BatteryLogger.NUM_CHANNELS; i++) {
            int portId = i;

            electricalAppliances[i] = new ElectricalAppliance(portId, Amps::zero, "Channel " + portId);
        }
    }

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
     *
     * <h2>Updates the battery simulation.</h2>
     *
     * <p>Calculates the battery voltage based on the current drawn by all appliances.
     *
     * <p>The battery voltage is clamped to avoid going below the brownout voltage.
     */
    public void simulationSubTick() {
        double totalCurrentAmps = getTotalCurrentDrawn().in(Amps);
        totalCurrentAmps = currentFilter.calculate(totalCurrentAmps);

        batteryVoltageVolts = Volts.of(
            BatterySim.calculateLoadedBatteryVoltage(BATTERY_NOMINAL_VOLTAGE.in(Volts), 0.02, totalCurrentAmps)
        );

        if (Double.isNaN(batteryVoltageVolts.in(Volts)) || Double.isInfinite(batteryVoltageVolts.in(Volts))) {
            batteryVoltageVolts = Volts.of(12.0);
            DriverStation.reportError(
                "[MapleSim] Internal Library Error: Calculated battery voltage is invalid" +
                ", reverting to normal operation voltage...",
                false
            );
        }
        if (batteryVoltageVolts.lt(Volts.of(RoboRioSim.getBrownoutVoltage()))) {
            batteryVoltageVolts = Volts.of(RoboRioSim.getBrownoutVoltage());
            DriverStation.reportError("[MapleSim] BrownOut Detected, protecting battery voltage...", false);
        }

        RoboRioSim.setVInVoltage(batteryVoltageVolts.in(Volts));
        // SmartDashboard.putNumber("BatterySim/TotalCurrent (Amps)", totalCurrentAmps);
        // SmartDashboard.putNumber("BatterySim/BatteryVoltage (Volts)", batteryVoltageVolts);
    }

    /**
     *
     *
     * <h2>Obtains the voltage of the battery.</h2>
     *
     * @return The battery voltage as a {@link Voltage} object.
     */
    public Voltage getBatteryVoltage() {
        return batteryVoltageVolts;
    }

    /**
     *
     *
     * <h2>Obtains the total current drawn from the battery.</h2>
     *
     * <p>Iterates through all the appliances to obtain the total current used.
     *
     * @return The total current as a {@link Current} object.
     */
    public Current getTotalCurrentDrawn() {
        // double totalCurrentAmps = electricalAppliances
        //     .stream()
        //     .mapToDouble(currentSupplier -> currentSupplier.get().in(Amps))
        //     .sum();
        return Amps.of(0);
    }

    /**
     *
     *
     * <h2>Clamps the voltage according to the supplied voltage and the battery's capabilities.</h2>
     *
     * <p>If the supplied voltage exceeds the battery's maximum voltage, it will be reduced to match the battery's
     * voltage.
     *
     * @param voltage The voltage to be clamped.
     * @return The clamped voltage as a {@link Voltage} object.
     */
    public Voltage clamp(Voltage voltage) {
        return Volts.of(
            MathUtil.clamp(voltage.in(Volts), -batteryVoltageVolts.in(Volts), batteryVoltageVolts.in(Volts))
        );
    }
}
