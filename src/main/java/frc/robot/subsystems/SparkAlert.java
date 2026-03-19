package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import java.util.function.Supplier;

public class SparkAlert extends DeviceAlert {

    private final Supplier<SparkMotorControllerData> dataSupplier;
    private final StringBuilder alertMessageBuilder;
    private final int initialMessageLength;

    public SparkAlert(Supplier<SparkMotorControllerData> dataSupplier, int canId, String deviceName) {
        super(canId, deviceName);
        this.dataSupplier = dataSupplier;

        alertMessageBuilder = new StringBuilder("SparkMax " + deviceName + " (ID " + Integer.toString(canId) + "): ");
        this.initialMessageLength = alertMessageBuilder.length();
    }

    private void buildFaultMessage(SparkBase.Faults faults) {
        alertMessageBuilder.append("[Faults: ");

        if (faults.motorType) {
            alertMessageBuilder.append("MotorType, ");
        }
        if (faults.sensor) {
            alertMessageBuilder.append("Sensor, ");
        }
        if (faults.can) {
            alertMessageBuilder.append("CAN, ");
        }
        if (faults.temperature) {
            alertMessageBuilder.append("Temperature, ");
        }
        if (faults.gateDriver) {
            alertMessageBuilder.append("GateDriver, ");
        }
        // TODO: what does this mean
        if (faults.escEeprom) {
            alertMessageBuilder.append("ESC_EEPROM, ");
        }
        if (faults.firmware) {
            alertMessageBuilder.append("Firmware, ");
        }
        if (faults.other) {
            alertMessageBuilder.append("Other, ");
        }

        // Remove trailing comma and space
        if (alertMessageBuilder.charAt(alertMessageBuilder.length() - 1) == ' ') {
            alertMessageBuilder.setLength(alertMessageBuilder.length() - 2);
        }
        alertMessageBuilder.append("]");
    }

    private void buildWarningMessage(SparkBase.Warnings warnings) {
        alertMessageBuilder.append("[Warnings: ");

        if (warnings.brownout) {
            alertMessageBuilder.append("Brownout, ");
        }
        if (warnings.overcurrent) {
            alertMessageBuilder.append("Overcurrent, ");
        }
        if (warnings.escEeprom) {
            alertMessageBuilder.append("ESC_EEPROM, ");
        }
        if (warnings.extEeprom) {
            alertMessageBuilder.append("ExternalEEPROM, ");
        }
        if (warnings.sensor) {
            alertMessageBuilder.append("Sensor, ");
        }
        if (warnings.stall) {
            alertMessageBuilder.append("Stall, ");
        }
        if (warnings.hasReset) {
            alertMessageBuilder.append("HasReset, ");
        }
        if (warnings.other) {
            alertMessageBuilder.append("Other, ");
        }

        // Remove trailing comma and space
        if (alertMessageBuilder.charAt(alertMessageBuilder.length() - 1) == ' ') {
            alertMessageBuilder.setLength(alertMessageBuilder.length() - 2);
        }
        alertMessageBuilder.append("]");
    }

    @Override
    public void updateConnectionStatus(boolean currentlyConnected) {
        SparkMotorControllerData data = dataSupplier.get();

        boolean hasAnyFault = SparkMotorControllerData.hasFault(data.getFaults());
        boolean hasAnyWarning = SparkMotorControllerData.hasWarning(data.getWarnings());

        currentlyConnected = currentlyConnected && !hasAnyFault && !hasAnyWarning;

        if (!currentlyConnected) {
            alertMessageBuilder.setLength(initialMessageLength);
            buildFaultMessage(data.getFaults());
            alertMessageBuilder.append(" ");
            buildWarningMessage(data.getWarnings());

            super.disconnectionAlert.setText(alertMessageBuilder.toString());
        }

        super.updateConnectionStatus(currentlyConnected);
    }
}
