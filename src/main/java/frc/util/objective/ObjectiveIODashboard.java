package frc.util.objective;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ObjectiveIODashboard implements ObjectiveIO {

    private String currentGameData = "";
    private ObjectiveIO.ActiveHub initialAlliance = ObjectiveIO.ActiveHub.ALL;
    private ObjectiveIO.ActiveHub oppositeAlliance = ObjectiveIO.ActiveHub.ALL;

    private final Timer matchTimer = new Timer();

    // Dashboard keys
    public static final String OBJECTIVE_DASHBOARD_KEY = "/ObjectiveTracker";
    public static final String DASHBOARD_PUBLISH_OBJECTIVE_KEY = "PublishObjective";

    private final SendableChooser<ActiveHub> activeOverrideChooser = new SendableChooser<>();

    private final NetworkTable objectiveTable;

    // Subscribers/publishers
    private final StringSubscriber publishObjectiveSubscriber;

    public ObjectiveIODashboard() {
        // Initialize tables
        objectiveTable = NetworkTableInstance.getDefault().getTable(OBJECTIVE_DASHBOARD_KEY);

        publishObjectiveSubscriber = objectiveTable.getStringTopic(DASHBOARD_PUBLISH_OBJECTIVE_KEY).subscribe("");

        // Initialize shift mode chooser with default values
        for (ActiveHub mode : ActiveHub.values()) {
            activeOverrideChooser.addOption(mode.toString(), mode);
        }

        activeOverrideChooser.setDefaultOption(ActiveHub.ALL.toString(), ActiveHub.ALL);

        SmartDashboard.putData("InitialActiveHubOverride", activeOverrideChooser);
    }

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
    public void resetForDisabled() {
        matchTimer.reset();
        matchTimer.stop();
    }

    @Override
    public void updateInputs(ObjectiveIOInputs inputs) {
        // if (publishObjectiveSubscriber.readQueue().length > 0) {
        //     inputs.sentObjective = publishObjectiveSubscriber.get();
        // }

        boolean isBeingOverrided = false;
        ActiveHub overrideSelection = activeOverrideChooser.getSelected();
        if (overrideSelection != null && overrideSelection != ActiveHub.ALL) {
            inputs.overrideInitialActiveHub = overrideSelection;
            initialAlliance = overrideSelection;
            inputs.initialActiveHub = overrideSelection;

            oppositeAlliance = overrideSelection == ActiveHub.RED ? ActiveHub.BLUE : ActiveHub.RED;
            isBeingOverrided = true;
        }

        // Only update if timer has started
        if (!matchTimer.isRunning()) {
            return;
        }

        if (DriverStation.isAutonomous()) {
            inputs.activeHub = ObjectiveIO.ActiveHub.ALL;
            inputs.currentShiftMode = ObjectiveIO.switches[0];
            inputs.nextShiftMode = ObjectiveIO.switches[1];

            inputs.timeUntilSwitch.mut_replace(
                Math.abs(ObjectiveIO.switches[0].startTimeSeconds) - matchTimer.get(),
                Seconds
            );
            inputs.timeElapsedFromLastSwitch.mut_replace(matchTimer.get(), Seconds);
            return;
        }

        // Set game data if we don't have it yet
        if (!isBeingOverrided && currentGameData.isEmpty()) {
            currentGameData = DriverStation.getGameSpecificMessage();

            // Set initial alliance if we don't have it yet
            if (!currentGameData.isEmpty()) {
                initialAlliance = currentGameData.charAt(0) == 'R'
                    ? ObjectiveIO.ActiveHub.RED
                    : ObjectiveIO.ActiveHub.BLUE;

                oppositeAlliance = currentGameData.charAt(0) == 'R'
                    ? ObjectiveIO.ActiveHub.BLUE
                    : ObjectiveIO.ActiveHub.RED;

                inputs.initialActiveHub = initialAlliance;
            }
        }

        // Determine active hub based on initial alliance and time since teleop
        double timeSinceTeleopSeconds = matchTimer.get();

        ShiftMode activeSwitchSetting = switches[switches.length - 1];
        ShiftMode nextSwitchSetting = switches[switches.length - 1];

        for (int i = 0; i < switches.length; i++) {
            ShiftMode setting = switches[i];

            // Find the first setting where the time since teleop is less than the runs until time
            if (timeSinceTeleopSeconds < setting.endTimeSeconds) {
                activeSwitchSetting = setting;

                // Determine the next switch setting
                if (i + 1 < switches.length) {
                    nextSwitchSetting = switches[i + 1];
                } else {
                    nextSwitchSetting = switches[switches.length - 1];
                }
                break;
            }
        }

        // Set active hub based on switch setting
        inputs.activeHub = activeSwitchSetting.active.toActiveHub(inputs.initialActiveHub);

        inputs.currentShiftMode = activeSwitchSetting;
        inputs.nextShiftMode = nextSwitchSetting;

        inputs.timeUntilSwitch.mut_replace(activeSwitchSetting.endTimeSeconds - timeSinceTeleopSeconds, Seconds);
        inputs.timeElapsedFromLastSwitch.mut_replace(
            timeSinceTeleopSeconds - activeSwitchSetting.startTimeSeconds,
            Seconds
        );
    }
}
