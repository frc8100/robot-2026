package frc.robot.subsystems;

import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import java.util.function.Supplier;

public class SparkAlert extends DeviceAlert {

    private final Supplier<SparkMotorControllerData> dataSupplier;
    private final StringBuilder alertMessageBuilder;

    public SparkAlert(Supplier<SparkMotorControllerData> dataSupplier, int canId, String deviceName) {
        super(canId, deviceName);
        this.dataSupplier = dataSupplier;

        alertMessageBuilder = new StringBuilder(super.disconnectionAlert.getText());
    }

    @Override
    public void updateConnectionStatus(boolean currentlyConnected) {
        SparkMotorControllerData data = dataSupplier.get();

        // TODO: change alert message to include more information about the device status

        super.updateConnectionStatus(currentlyConnected);
    }
}
