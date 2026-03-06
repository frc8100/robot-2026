package frc.util.objective;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectiveIO {
    /**
     * The hub that is currently active.
     */
    public enum ActiveHub {
        /**
         * All hubs are active, during auto and endgame.
         */
        ALL("FMS Determination", Color.kFloralWhite),

        /**
         * The red alliance hub is active.
         */
        RED("Red", Color.kRed),

        /**
         * The blue alliance hub is active.
         */
        BLUE("Blue", Color.kBlue);

        public final String displayNameWhenOverriding;
        public final Color color;

        private ActiveHub(String displayNameWhenOverriding, Color color) {
            this.displayNameWhenOverriding = displayNameWhenOverriding;
            this.color = color;
        }

        /**
         * Converts a {@link DriverStation.Alliance} to an {@link ActiveHub}.
         * @param alliance - The alliance to convert.
         * @return The corresponding ActiveHub.
         */
        public static ActiveHub fromAlliance(DriverStation.Alliance alliance) {
            switch (alliance) {
                case Red:
                    return RED;
                case Blue:
                    return BLUE;
                default:
                    return ALL;
            }
        }

        /**
         * Converts a {@link DriverStation.Alliance} to the opposite {@link ActiveHub}.
         * @param alliance - The alliance to convert.
         * @return The corresponding ActiveHub.
         */
        public static ActiveHub fromOppositeAlliance(DriverStation.Alliance alliance) {
            switch (alliance) {
                case Red:
                    return BLUE;
                case Blue:
                    return RED;
                default:
                    return ALL;
            }
        }

        /**
         * Checks if the given hub is active for the given alliance.
         * @param hub - The hub to check.
         * @param alliance - The alliance to check.
         * @return True if the hub is active for the alliance, false otherwise.
         */
        public static boolean isActive(ActiveHub hub, DriverStation.Alliance alliance) {
            switch (hub) {
                case ALL:
                    return true;
                case RED:
                    return alliance.equals(DriverStation.Alliance.Red);
                case BLUE:
                    return alliance.equals(DriverStation.Alliance.Blue);
                default:
                    return false;
            }
        }

        /**
         * Checks if this hub is active for the given alliance.
         * @param alliance - The alliance to check.
         * @return True if this hub is active for the alliance, false otherwise.
         */
        public boolean isActive(DriverStation.Alliance alliance) {
            return isActive(this, alliance);
        }

        @Override
        public String toString() {
            return displayNameWhenOverriding;
        }
    }

    /**
     * Represents which alliance's hub is active during a shift mode.
     */
    public enum ShiftModeAllianceActive {
        ALL_ACTIVE,
        INITIAL_ACTIVE,
        OPPOSITE_ACTIVE,
    }

    /**
     * Represents a shift mode, which determines which alliance's hub is active and when the next shift will occur.
     * @param name - The name of the shift mode, for logging purposes.
     * @param runsUntilTimeSeconds - The time in seconds until the next shift occurs,
     * @param active - Which alliance's hub is active during this shift mode.
     */
    public record ShiftMode(String name, double runsUntilTimeSeconds, ShiftModeAllianceActive active) {
        @Override
        public final String toString() {
            return name;
        }
    }

    /**
     * The shift modes for teleop, in order.
     */
    public static final ShiftMode[] switches = new ShiftMode[] {
        // Transition shift
        new ShiftMode("Transition Shift", 10.0, ShiftModeAllianceActive.ALL_ACTIVE),
        // Alliance shifts every 25s
        new ShiftMode("Shift 1", 35.0, ShiftModeAllianceActive.INITIAL_ACTIVE),
        new ShiftMode("Shift 2", 60.0, ShiftModeAllianceActive.OPPOSITE_ACTIVE),
        new ShiftMode("Shift 3", 85.0, ShiftModeAllianceActive.INITIAL_ACTIVE),
        new ShiftMode("Shift 4", 110.0, ShiftModeAllianceActive.OPPOSITE_ACTIVE),
        // Endgame
        new ShiftMode("Endgame", 140.0, ShiftModeAllianceActive.ALL_ACTIVE),
    };

    @AutoLog
    public static class ObjectiveIOInputs {

        /**
         * The hub that is currently active.
         */
        public ActiveHub activeHub = ActiveHub.ALL;

        /**
         * The hub that is initially active at the start of teleop, if overridden by the operator.
         * If this is {@link ActiveHub#ALL}, the initial active hub will be determined by the winner of auto broadcasted by the FMS.
         */
        public ActiveHub overrideInitialActiveHub = ActiveHub.ALL;

        /**
         * The time remaining in the current period.
         * Counts down to zero. Should always be non-negative.
         */
        public MutTime timeUntilSwitch = Seconds.mutable(0.0);
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(ObjectiveIOInputs inputs) {}

    /**
     * Called at the start of auto {@link frc.robot.Robot#autonomousInit} to reset any necessary state.
     */
    public default void resetForAuto() {}

    /**
     * Called at the start of teleop {@link frc.robot.Robot#teleopInit} to reset any necessary state.
     */
    public default void resetForTeleop() {}
}
