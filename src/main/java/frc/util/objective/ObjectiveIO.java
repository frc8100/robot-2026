package frc.util.objective;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectiveIO {
    /**
     * The hub that is currently active.
     */
    public enum ActiveHub {
        /**
         * All hubs are active, during auto and endgame.
         */
        ALL,

        /**
         * The red alliance hub is active.
         */
        RED,

        /**
         * The blue alliance hub is active.
         */
        BLUE;

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
    }

    @AutoLog
    public static class ObjectiveIOInputs {

        /**
         * The hub that is currently active.
         */
        public ActiveHub activeHub = ActiveHub.ALL;

        /**
         * The time remaining in the current period.
         * Counts down to zero. Should always be non-negative.
         */
        public MutTime timeUntilSwitch = Seconds.mutable(0.0);
    }

    public default void updateInputs(ObjectiveIOInputs inputs) {}
}
