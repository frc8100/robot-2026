package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Joules;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.units.measure.Energy;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutEnergy;
import edu.wpi.first.units.measure.MutPower;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANIdConstants;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class BatteryLogger extends SubsystemBase {

    public static final int NUM_CHANNELS = 24;

    @AutoLog
    public static class BatteryLoggerIOInputs {

        public double[] channelCurrentsAmps = new double[NUM_CHANNELS];
        public double totalCurrentAmps = 0.0;
        public double currentBatteryVoltage = 12.0;
    }

    private final BatteryLoggerIOInputsAutoLogged inputs = new BatteryLoggerIOInputsAutoLogged();

    private final PowerDistribution powerDistribution = new PowerDistribution(
        CANIdConstants.POWER_DISTRIBUTION_ID,
        PowerDistribution.ModuleType.kRev
    );

    private final MutCurrent accumulatedCurrent = Amps.mutable(0.0);
    private final MutPower accumulatedPower = Watts.mutable(0.0);
    private final MutEnergy accumulatedEnergy = Joules.mutable(0.0);

    public BatteryLogger() {}

    /**
     * @return The power (wattage) accumulated from the battery since the robot was enabled.
     */
    @AutoLogOutput(key = "BatteryLogger/Accumulated/Power")
    public Power getAccumulatedPower() {
        return accumulatedPower;
    }

    /**
     * @return The current (amperage) accumulated from the battery since the robot was enabled.
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

    @Override
    public void periodic() {
        // TODO: IO implementation
        // TODO: reduce allocation by reusing existing array
        inputs.channelCurrentsAmps = powerDistribution.getAllCurrents();
        inputs.totalCurrentAmps = powerDistribution.getTotalCurrent();

        Logger.processInputs("BatteryLogger", inputs);

        // Update accumulated energy and current
        accumulatedCurrent.mut_plus(Amps.of(inputs.totalCurrentAmps));
        accumulatedPower.mut_plus(getInstantaneousPower());
        accumulatedEnergy.mut_plus(getInstantaneousEnergy());
    }
}
