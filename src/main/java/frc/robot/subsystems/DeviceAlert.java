package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Contains an alert for a specific CAN ID to monitor its connection status.
 */
public class DeviceAlert {

    public enum DeviceAlertType {
        CAN_ID,
        NETWORK_TABLE,
    }

    /**
     * The default debounce time for CAN connection alerts.
     */
    public static final Time DEFAULT_DEBOUNCE_TIME = Seconds.of(0.5);

    /**
     * The default alert type for CAN connection alerts.
     * Currently set to {@link AlertType#kWarning} to make CAN bus disruptions (several disconnected devices) able to be set to {@link AlertType#kError} to make them more visible.
     */
    public static final AlertType DEFAULT_ALERT_TYPE = AlertType.kWarning;

    /**
     * A debouncer to prevent alert flapping.
     */
    private final Debouncer connectionDebouncer = new Debouncer(
        DEFAULT_DEBOUNCE_TIME.in(Seconds),
        Debouncer.DebounceType.kFalling
    );

    /**
     * The type of alert (CAN ID or Network Table) for this device alert.
     */
    public final DeviceAlertType alertType;

    /**
     * The alert to be triggered on disconnection.
     */
    private final Alert disconnectionAlert;

    /**
     * The CAN ID of the device being monitored.
     * If {@link #alertType} is {@link DeviceAlertType#NETWORK_TABLE}, this will be set to -1 since it won't be used.
     */
    public final int canId;

    /**
     * The name of the device being monitored. Used in the alert message.
     */
    public final String deviceName;

    /**
     * The current connection status of the CAN device.
     * Set using a debouncer.
     */
    private boolean isConnected = true;

    /**
     * Creates a CAN ID alert for the given CAN ID and device name.
     * @param canId - The CAN ID of the device.
     * @param deviceName - The name of the device. Used in the alert message.
     */
    public DeviceAlert(int canId, String deviceName) {
        this.deviceName = deviceName;
        this.alertType = DeviceAlertType.CAN_ID;

        this.canId = canId;

        this.disconnectionAlert = new Alert(
            CANIdConnections.CAN_ID_ALERT_GROUP,
            "Disconnected CAN device: " + deviceName + " (ID " + Integer.toString(canId) + ").",
            DEFAULT_ALERT_TYPE
        );

        CANIdConnections.registerCANIdAlert(this);
    }

    /**
     * Creates a Network alert for the given device name.
     * @param networkDeviceName - The name of the network device. Used in the alert message.
     */
    public DeviceAlert(String networkDeviceName) {
        this.deviceName = networkDeviceName;
        this.alertType = DeviceAlertType.NETWORK_TABLE;

        this.canId = -1;

        this.disconnectionAlert = new Alert(
            CANIdConnections.NETWORK_ALERT_GROUP,
            "Disconnected Network device: " + networkDeviceName + ".",
            DEFAULT_ALERT_TYPE
        );
    }

    /**
     * Updates the connection status of the CAN device. Sets the alert if disconnected.
     * @param currentlyConnected - Whether the device is currently connected.
     */
    public void updateConnectionStatus(boolean currentlyConnected) {
        this.isConnected = connectionDebouncer.calculate(currentlyConnected);

        disconnectionAlert.set(!isConnected);
    }

    /**
     * Returns whether the CAN device is currently connected.
     * @return True if connected, false otherwise.
     */
    public boolean isConnected() {
        return isConnected;
    }
}
