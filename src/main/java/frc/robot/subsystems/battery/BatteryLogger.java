package frc.robot.subsystems.battery;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Joules;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;
import static edu.wpi.first.units.Units.derive;

import edu.wpi.first.units.EnergyUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Energy;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class BatteryLogger extends SubsystemBase {

    public static final int NUM_CHANNELS = 24;

    public static final EnergyUnit WattHours = derive(Joules).aggregate(3600).named("Watt-Hour").symbol("Wh").make();

    private final BatteryIO io;
    private final BatteryIOInputsAutoLogged inputs = new BatteryIOInputsAutoLogged();

    private Current accumulatedCurrent = Amps.zero();
    private Power accumulatedPower = Watts.zero();
    private Energy accumulatedEnergy = Joules.zero();

    public BatteryLogger(BatteryIO io) {
        this.io = io;
    }

    /**
     * @return The power (wattage) accumulated from the battery since the robot was enabled.
     */
    @AutoLogOutput(key = "BatteryLogger/Accumulated/Power")
    public Power getAccumulatedPower() {
        return accumulatedPower;
    }

    /**
     * @return The energy (joules/watt-hours) accumulated from the battery since the robot was enabled.
     */
    @AutoLogOutput(key = "BatteryLogger/Accumulated/Energy")
    public Energy getAccumulatedEnergy() {
        return accumulatedEnergy;
    }

    /**
     * @return The power (wattage) being drawn from the battery at this loop cycle.
     */
    @AutoLogOutput(key = "BatteryLogger/Instant/Power")
    public Power getInstantaneousPower() {
        return Amps.of(inputs.totalCurrentAmps).times(Volts.of(inputs.currentBatteryVoltage));
    }

    /**
     * @return The energy (joules/watt-hours) being drawn from the battery at this loop cycle.
     */
    @AutoLogOutput(key = "BatteryLogger/Instant/Energy")
    public Energy getInstantaneousEnergy() {
        Power power = getInstantaneousPower();
        return power.times(Constants.LOOP_PERIOD_TIME);
    }

    /**
     * Resets the accumulated energy and current values to zero.
     */
    // TODO: call this
    public void resetAccumulated() {
        accumulatedCurrent = Amps.zero();
        accumulatedPower = Watts.zero();
        accumulatedEnergy = Joules.zero();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("BatteryLogger", inputs);

        // Update accumulated energy and current
        accumulatedCurrent = accumulatedCurrent.plus(Amps.of(inputs.totalCurrentAmps));
        accumulatedPower = accumulatedPower.plus(getInstantaneousPower());
        accumulatedEnergy = accumulatedEnergy.plus(getInstantaneousEnergy());
    }
}
