package frc.robot.subsystems.battery;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.CANIdConstants;

public class BatteryIOReal implements BatteryIO {

    private final PowerDistribution powerDistribution = new PowerDistribution(
        CANIdConstants.POWER_DISTRIBUTION_ID,
        PowerDistribution.ModuleType.kRev
    );

    @Override
    public void updateInputs(BatteryIOInputs inputs) {
        // TODO: reduce allocation by reusing existing array
        inputs.channelCurrentsAmps = powerDistribution.getAllCurrents();
        inputs.totalCurrentAmps = powerDistribution.getTotalCurrent();

        // TODO: Difference between this and RobotController.getBatteryVoltage()?
        inputs.currentBatteryVoltage = powerDistribution.getVoltage();
    }
}
