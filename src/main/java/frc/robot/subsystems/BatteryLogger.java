package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.CANIdConstants;
import org.littletonrobotics.junction.AutoLog;

public class BatteryLogger {

    @AutoLog
    public static class BatteryLoggerIOInputs {

        public double[] channelCurrents = new double[24];
    }

    private final PowerDistribution powerDistribution = new PowerDistribution(
        CANIdConstants.POWER_DISTRIBUTION_ID,
        PowerDistribution.ModuleType.kRev
    );

    public BatteryLogger() {}
}
