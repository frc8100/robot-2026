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
        ALL("FMS Determination", Color.kLimeGreen),

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
        OPPOSITE_ACTIVE;

        public ActiveHub toActiveHub(ActiveHub initialActiveHub) {
            switch (this) {
                case ALL_ACTIVE:
                    return ActiveHub.ALL;
                case INITIAL_ACTIVE:
                    return initialActiveHub;
                case OPPOSITE_ACTIVE:
                    return initialActiveHub == ActiveHub.RED ? ActiveHub.BLUE : ActiveHub.RED;
                default:
                    return ActiveHub.ALL;
            }
        }
    }

    /**
     * Represents a shift mode, which determines which alliance's hub is active and when the next shift will occur.
     */
    public enum ShiftMode {
        AUTO("Auto", -20.0, 0.0, ShiftModeAllianceActive.ALL_ACTIVE),
        TRANSITION_SHIFT("Transition Shift", 0.0, 10.0, ShiftModeAllianceActive.ALL_ACTIVE),
        SHIFT_1("Shift 1", 10.0, 35.0, ShiftModeAllianceActive.INITIAL_ACTIVE),
        SHIFT_2("Shift 2", 35.0, 60.0, ShiftModeAllianceActive.OPPOSITE_ACTIVE),
        SHIFT_3("Shift 3", 60.0, 85.0, ShiftModeAllianceActive.INITIAL_ACTIVE),
        SHIFT_4("Shift 4", 85.0, 110.0, ShiftModeAllianceActive.OPPOSITE_ACTIVE),
        ENDGAME("Endgame", 110.0, 140.0, ShiftModeAllianceActive.ALL_ACTIVE);

        /**
         * The display name of the shift mode.
         */
        public final String displayName;

        /**
         * The start time of the shift mode in seconds.
         */
        public final double startTimeSeconds;

        /**
         * The end time of the shift mode in seconds.
         */
        public final double endTimeSeconds;

        /**
         * The alliance that is active during this shift mode.
         */
        public final ShiftModeAllianceActive active;

        private ShiftMode(
            String displayName,
            double startTimeSeconds,
            double endTimeSeconds,
            ShiftModeAllianceActive active
        ) {
            this.displayName = displayName;
            this.startTimeSeconds = startTimeSeconds;
            this.endTimeSeconds = endTimeSeconds;
            this.active = active;
        }

        @Override
        public String toString() {
            return displayName;
        }
    }

    /**
     * The shift modes for teleop, in order.
     */
    // public static final List<ShiftMode> switches = new LinkedList<>(List.of(ShiftMode.values()));
    // public static final ListIterator<ShiftMode> switchIterator = switches.listIterator();
    public static final ShiftMode[] switches = ShiftMode.values();

    public static ShiftMode getShiftMode(double timeSeconds) {
        for (ShiftMode shiftMode : switches) {
            if (timeSeconds >= shiftMode.startTimeSeconds && timeSeconds < shiftMode.endTimeSeconds) {
                return shiftMode;
            }
        }

        // Return the last shift mode if time is beyond all defined shifts
        return switches[switches.length - 1];
    }

    public static ShiftMode getShiftModeFromTimeOfCount(double timeOfFuelCount) {
        for (ShiftMode shiftMode : switches) {
            if (
                timeOfFuelCount >= shiftMode.startTimeSeconds &&
                timeOfFuelCount <
                (shiftMode.endTimeSeconds + ObjectiveTracker.TIME_AFTER_SHIFT_ENDS_WHERE_HUB_STILL_COUNTS.in(Seconds))
            ) {
                return shiftMode;
            }
        }

        // Return the last shift mode if time is beyond all defined shifts
        return switches[switches.length - 1];
    }

    @AutoLog
    public static class ObjectiveIOInputs {

        /**
         * The current shift mode.
         */
        public ShiftMode currentShiftMode = switches[0];

        /**
         * The next shift mode.
         */
        public ShiftMode nextShiftMode = switches[1];

        /**
         * The hub that is currently active.
         */
        public ActiveHub activeHub = ActiveHub.ALL;

        /**
         * The hub that is initially active in shift 1.
         */
        public ActiveHub initialActiveHub = ActiveHub.ALL;

        /**
         * The hub that is initially active at the start of teleop, if overridden by the operator.
         * If this is {@link ActiveHub#ALL}, the initial active hub will be determined by the winner of auto broadcasted by the FMS.
         */
        public ActiveHub overrideInitialActiveHub = ActiveHub.ALL;

        /**
         * The time remaining in the current period.
         * Counts down to zero. Should always be non-negative.
         */
        // Start at 20 for auto in case match timer doesn't work
        public MutTime timeUntilSwitch = Seconds.mutable(20.0);

        /**
         * The time elapsed since the last shift occurred.
         * Counts up from zero. Should always be non-negative.
         */
        public MutTime timeElapsedFromLastSwitch = Seconds.mutable(0.0);
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

    /**
     * Called at the start of disabled {@link frc.robot.Robot#disabledInit} to reset any necessary state.
     */
    public default void resetForDisabled() {}
}
