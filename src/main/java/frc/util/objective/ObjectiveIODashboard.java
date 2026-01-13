package frc.util.objective;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

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
            if (timeSinceTeleopSeconds < setting.runsUntilTimeSeconds) {
                activeSwitchSetting = setting;
            }
        }

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
