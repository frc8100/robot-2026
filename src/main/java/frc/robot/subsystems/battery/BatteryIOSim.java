package frc.robot.subsystems.battery;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class BatteryIOSim implements BatteryIO {

    @Override
    public void updateInputs(BatteryIOInputs inputs) {
        // TODO: calculate channel currents

        inputs.totalCurrentAmps = SimulatedBattery.getTotalCurrentDrawn().in(Amps);
        inputs.currentBatteryVoltage = SimulatedBattery.getBatteryVoltage().in(Volts);
    }
}
