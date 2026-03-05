package frc.util.objective;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class ObjectiveIODashboard implements ObjectiveIO {

    private String currentGameData = "";
    private ObjectiveIO.ActiveHub initialAlliance = ObjectiveIO.ActiveHub.ALL;
    private ObjectiveIO.ActiveHub oppositeAlliance = ObjectiveIO.ActiveHub.ALL;

    private final Timer matchTimer = new Timer();

    // Dashboard keys
    public static final String OBJECTIVE_DASHBOARD_KEY = "/ObjectiveTracker";
    public static final String DASHBOARD_PUBLISH_OBJECTIVE_KEY = "PublishObjective";

    // TODO: this doesnt have to be Logged because this is in an IO implementation
    private final LoggedDashboardChooser<ActiveHub> activeOverrideChoose = new LoggedDashboardChooser<>(
        "InitialActiveHubOverride"
    );

    private final NetworkTable objectiveTable;

    // Subscribers/publishers
    private final StringSubscriber publishObjectiveSubscriber;

    public ObjectiveIODashboard() {
        // Initialize tables
        objectiveTable = NetworkTableInstance.getDefault().getTable(OBJECTIVE_DASHBOARD_KEY);

        publishObjectiveSubscriber = objectiveTable.getStringTopic(DASHBOARD_PUBLISH_OBJECTIVE_KEY).subscribe("");

        // Initialize shift mode chooser with default values
        for (ActiveHub mode : ActiveHub.values()) {
            activeOverrideChoose.getSendableChooser().addOption(mode.name(), mode.toString());
        }
    }

    // TODO: call these methods
    @Override
    public void resetForAuto() {
        matchTimer.reset();
        matchTimer.start();
    }

    @Override
    public void resetForTeleop() {
        resetForAuto();
    }

    @Override
    public void updateInputs(ObjectiveIOInputs inputs) {
        // if (publishObjectiveSubscriber.readQueue().length > 0) {
        //     inputs.sentObjective = publishObjectiveSubscriber.get();
        // }

        // Only update if timer has started
        if (!matchTimer.isRunning()) {
            return;
        }

        if (DriverStation.isAutonomous()) {
            inputs.activeHub = ObjectiveIO.ActiveHub.ALL;
            return;
        }

        // Set game data if we don't have it yet
        if (currentGameData.isEmpty()) {
            currentGameData = DriverStation.getGameSpecificMessage();

            // Set initial alliance if we don't have it yet
            if (!currentGameData.isEmpty()) {
                initialAlliance = currentGameData.charAt(0) == 'R'
                    ? ObjectiveIO.ActiveHub.RED
                    : ObjectiveIO.ActiveHub.BLUE;

                oppositeAlliance = currentGameData.charAt(0) == 'R'
                    ? ObjectiveIO.ActiveHub.BLUE
                    : ObjectiveIO.ActiveHub.RED;
            }
        }

        // Determine active hub based on initial alliance and time since teleop
        double timeSinceTeleopSeconds = matchTimer.get();

        ShiftMode activeSwitchSetting = switches[switches.length - 1];

        for (ShiftMode setting : switches) {
            // Find the first setting where the time since teleop is less than the runs until time
            if (timeSinceTeleopSeconds < setting.runsUntilTimeSeconds()) {
                activeSwitchSetting = setting;
                break;
            }
        }

        // Set active hub based on switch setting
        switch (activeSwitchSetting.active()) {
            case INITIAL_ACTIVE:
                inputs.activeHub = initialAlliance;
                break;
            case OPPOSITE_ACTIVE:
                inputs.activeHub = oppositeAlliance;
                break;
            case ALL_ACTIVE:
            default:
                inputs.activeHub = ObjectiveIO.ActiveHub.ALL;
                break;
        }

        inputs.timeUntilSwitch.mut_replace(
            activeSwitchSetting.runsUntilTimeSeconds() - timeSinceTeleopSeconds,
            Seconds
        );
    }
}
