package frc.util.objective;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class ObjectiveIODashboard implements ObjectiveIO {

    private enum SwitchSettingAllianceActive {
        ALL_ACTIVE,
        INITIAL_ACTIVE,
        OPPOSITE_ACTIVE,
    }

    public record SwitchSetting(double runsUntilTimeSeconds, SwitchSettingAllianceActive active) {}

    private static final SwitchSetting[] switches = new SwitchSetting[] {
        // Transition shift
        new SwitchSetting(10.0, SwitchSettingAllianceActive.ALL_ACTIVE),
        // Alliance shifts every 25s
        new SwitchSetting(35.0, SwitchSettingAllianceActive.INITIAL_ACTIVE),
        new SwitchSetting(60.0, SwitchSettingAllianceActive.OPPOSITE_ACTIVE),
        new SwitchSetting(85.0, SwitchSettingAllianceActive.INITIAL_ACTIVE),
        new SwitchSetting(110.0, SwitchSettingAllianceActive.OPPOSITE_ACTIVE),
        // Endgame
        new SwitchSetting(140.0, SwitchSettingAllianceActive.ALL_ACTIVE),
    };

    private String currentGameData = "";
    private ObjectiveIO.ActiveHub initialAlliance = ObjectiveIO.ActiveHub.ALL;
    private ObjectiveIO.ActiveHub oppositeAlliance = ObjectiveIO.ActiveHub.ALL;

    private final Timer matchTimer = new Timer();

    // Dashboard keys
    public static final String OBJECTIVE_DASHBOARD_KEY = "/ObjectiveTracker";
    public static final String DASHBOARD_PUBLISH_OBJECTIVE_KEY = "PublishObjective";

    private final NetworkTable objectiveTable;

    // Subscribers/publishers
    private final StringSubscriber publishObjectiveSubscriber;

    public ObjectiveIODashboard() {
        // Initialize tables
        objectiveTable = NetworkTableInstance.getDefault().getTable(OBJECTIVE_DASHBOARD_KEY);

        publishObjectiveSubscriber = objectiveTable.getStringTopic(DASHBOARD_PUBLISH_OBJECTIVE_KEY).subscribe("");
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

        SwitchSetting activeSwitchSetting = switches[switches.length - 1];

        for (SwitchSetting setting : switches) {
            // Find the first setting where the time since teleop is less than the runs until time
            if (timeSinceTeleopSeconds < setting.runsUntilTimeSeconds) {
                activeSwitchSetting = setting;
                break;
            }
        }

        // Set active hub based on switch setting
        switch (activeSwitchSetting.active) {
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

        inputs.timeUntilSwitch.mut_replace(activeSwitchSetting.runsUntilTimeSeconds - timeSinceTeleopSeconds, Seconds);
    }
}
