package frc.robot.subsystems;

import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANIdConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class BatteryLogger {

    public static final int NUM_CHANNELS = 24;

    @AutoLog
    public static class BatteryLoggerIOInputs {

        public double[] channelCurrentsAmps = new double[NUM_CHANNELS];
        public double totalCurrentAmps = 0.0;
    }

    private final BatteryLoggerIOInputsAutoLogged inputs = new BatteryLoggerIOInputsAutoLogged();

    private final PowerDistribution powerDistribution = new PowerDistribution(
        CANIdConstants.POWER_DISTRIBUTION_ID,
        PowerDistribution.ModuleType.kRev
    );

    public BatteryLogger() {}

    public void periodic() {
        // TODO: IO implementation
        // TODO: reduce allocation by reusing existing array
        inputs.channelCurrentsAmps = powerDistribution.getAllCurrents();
        inputs.totalCurrentAmps = powerDistribution.getTotalCurrent();

        Logger.processInputs("BatteryLogger", inputs);
    }
}
