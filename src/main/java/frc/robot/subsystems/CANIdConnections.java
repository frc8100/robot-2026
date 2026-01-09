package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.CANIdConstants;
import java.util.ArrayList;
import java.util.List;

/**
 * Contains all the CAN IDs and utilities to detect connection disruptions with alerts.
 */
public class CANIdConnections {

    private CANIdConnections() {}

    /**
     * The alert group for CAN ID alerts.
     */
    public static final String ALERT_GROUP = "CANAlerts";

    /**
     * List of CAN ID alerts for monitoring CAN device connections.
     */
    private static final List<CANIdAlert> canIdAlerts = new ArrayList<>();

    /**
     * List of CAN IDs that were last detected as disconnected.
     */
    private static final List<Integer> lastDisconnectedIds = new ArrayList<>();

    /**
     * List of connection disruptions last detected.
     */
    private static final List<Integer> lastDisruptions = new ArrayList<>();

    /**
     * Initial part of the disruption message for {@link #disruptionMessageBuilder}.
     */
    private static final String INITIAL_DISRUPTION_MESSAGE = "CAN bus disruption detected at connections: ";

    /**
     * StringBuilder for building disruption messages.
     */
    private static final StringBuilder disruptionMessageBuilder = new StringBuilder(INITIAL_DISRUPTION_MESSAGE);

    /**
     * Registers a CAN ID alert to be monitored.
     * @param alert - The CAN ID alert to register.
     */
    public static void registerCANIdAlert(CANIdAlert alert) {
        canIdAlerts.add(alert);
    }

    /**
     * Alert for CAN bus disruptions (two or more consecutive disconnected CAN IDs).
     * Empty message; will be set in {@link #periodic()}
     */
    public static final Alert canBusDisruptionAlert = new Alert(ALERT_GROUP, "", AlertType.kError);

    /**
     * Gets a list connections that are disrupted based on the list of disconnected CAN IDs {@link #lastDisconnectedIds}.
     * {@link #lastDisruptions} is updated in place. {@link #lastDisconnectedIds} should be updated before calling this method.
     *
     * <p> A disruption is when two or more consecutive CAN IDs are disconnected.
     * If three or more consecutive CAN IDs are disconnected, that counts as one disruption starting at the first disconnected CAN ID.
     * A disruption at index `i` means that the connection between `canIdConnectionsInOrder[i - 1]` and `canIdConnectionsInOrder[i]` is disrupted.
     * (Connection 0 is between the PDP and the first CAN ID in the list.)
     * @return A list of indices where disruptions occur.
     */
    public static void updateLastDisruptions() {
        lastDisruptions.clear();

        /**
         * The number of consecutive disconnected CAN IDs seen so far.
         */
        int consecutive = 0;

        for (int i = 0; i < CANIdConstants.canIdConnectionsInOrder.length; i++) {
            int canId = CANIdConstants.canIdConnectionsInOrder[i];

            if (lastDisconnectedIds.contains(canId)) {
                consecutive++;

                // Start of a new disruption
                if (consecutive == 2) {
                    lastDisruptions.add(i);
                }
            } else {
                // Reset consecutive count
                consecutive = 0;
            }
        }
    }

    /**
     * Periodically checks the connection status of all registered CAN ID alerts and updates the CAN bus disruption alert accordingly.
     */
    public static void periodic() {
        lastDisconnectedIds.clear();
        for (CANIdAlert alert : canIdAlerts) {
            if (!alert.isConnected()) {
                // Add the CAN ID to the disconnected list
                lastDisconnectedIds.add(alert.canId);
            }
        }

        updateLastDisruptions();

        // If no disruptions, clear alert and return
        if (lastDisruptions.isEmpty()) {
            canBusDisruptionAlert.set(false);
            return;
        }

        // Reset disruption message builder
        disruptionMessageBuilder.setLength(INITIAL_DISRUPTION_MESSAGE.length());

        // Set alert based on disruptions
        for (int i = 0; i < lastDisruptions.size(); i++) {
            int disruptionIndex = lastDisruptions.get(i);

            disruptionMessageBuilder.append(disruptionIndex);

            if (i < lastDisruptions.size() - 1) {
                disruptionMessageBuilder.append(", ");
            }
        }

        // Finalize and set alert
        disruptionMessageBuilder.append(".");
        canBusDisruptionAlert.setText(disruptionMessageBuilder.toString());
        canBusDisruptionAlert.set(true);
    }
}
