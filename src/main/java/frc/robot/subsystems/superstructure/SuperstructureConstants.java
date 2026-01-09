package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.superstructure.claw.ClawConstants.RotationPositions.CLAW_ANGLE_OFFSET;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.Position.INITIAL_HEIGHT_CLAW;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.superstructure.claw.ClawConstants;
import frc.util.TunableValue;

/**
 * Contains the constants for the superstructure.
 */
public final class SuperstructureConstants {

    /**
     * The levels of the reef and the corresponding elevator height and claw angle.
     */
    public enum Level {
        /**
         * Also the initial position of the elevator and claw.
         */
        INITIAL_POSITION("L1", Meters.of(0.0), CLAW_ANGLE_OFFSET, 0),
        L2("L2", Meters.of(1).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(57.6), 6),
        L3("L3", Meters.of(1.25).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(57.5), 14),
        L4("L4", Meters.of(2.1).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(60.5), 29),

        L1_AUTO("L1Auto", Meters.of(0.4).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(180), 3.5),

        ALGAE_L2("L2Algae", Meters.of(0.9), Rotation2d.fromDegrees(180), 4.8),
        ALGAE_L3("L3Algae", Meters.of(1.2), Rotation2d.fromDegrees(180), 10),

        ALGAE_HOLD_NET("AlgaeNet", Meters.of(2.1).minus(INITIAL_HEIGHT_CLAW), Rotation2d.fromDegrees(180), 25); // TODO: increase height

        private final Distance elevatorDistance;

        /**
         * @return The distance of the elevator.
         * Measured from the ground to the top of the elevator.
         */
        public Distance getElevatorDistance() {
            return elevatorDistance;
        }

        private final Rotation2d clawAngle;

        /**
         * @return The angle of the claw.
         */
        public Rotation2d getClawAngle() {
            return clawAngle;
        }

        private final double elevatorRadian;
        private final TunableValue elevatorRadianTunable;

        /**
         * @return The radian of the elevator.
         */
        public double getElevatorRadian() {
            return elevatorRadian;
            // return elevatorRadianTunable.get();
        }

        /**
         * Creates a new level.
         * @param elevatorDistance - The distance to run the elevator.
         * @param clawAngle - The angle to run the claw, without the offset.
         */
        private Level(String key, Distance elevatorDistance, Rotation2d clawAngle, double elevatorRadian) {
            this.elevatorDistance = elevatorDistance;
            this.clawAngle = clawAngle.minus(CLAW_ANGLE_OFFSET);
            this.elevatorRadian = elevatorRadian;

            this.elevatorRadianTunable = new TunableValue("SSLevels/" + key, elevatorRadian);
        }
    }

    /**
     * The range below and above the critical levels to wait for the claw to rotate.
     * The elevator will move until it hits this range (assuming a critical level is between the current and target position).
     * It will then wait for the claw to rotate to the correct angle before moving again.
     */
    public static final Angle ELEVATOR_RADIAN_RANGE_BETWEEN_CRITICAL_LEVELS_TO_WAIT_FOR_CLAW = Radians.of(2.25);

    /**
     * A list of critical levels. See {@link CriticalLevel}.
     */
    public static final CriticalLevel[] CRITICAL_LEVELS = {
        new CriticalLevel(Radians.of(15), ClawConstants.RotationPositions.CLAW_HOLDING_POSITION),
        new CriticalLevel(Radians.of(18), ClawConstants.RotationPositions.CLAW_HOLDING_POSITION),
    };

    public static record CriticalLevelRaw(Angle elevatorRadian, Rotation2d clawAngle) {}

    /**
     * Represents points on the elevator position in which a specific claw rotation is needed to avoiding hitting the elevator.
     */
    public static class CriticalLevel {

        /**
         * The radian of the elevator at this critical level.
         */
        private final Angle elevatorAngle;

        /**
         * The angle of the claw to avoid hitting the elevator.
         */
        private final Rotation2d clawAngle;

        public Rotation2d getClawAngle() {
            return clawAngle;
        }

        public Angle getElevatorAngle() {
            return elevatorAngle;
        }

        /**
         * @return The radian of the elevator at the lower bound of the critical level. See {@link #ELEVATOR_RADIAN_RANGE_BETWEEN_CRITICAL_LEVELS_TO_WAIT_FOR_CLAW}
         */
        private Angle getLowerElevatorRadian() {
            return elevatorAngle.minus(ELEVATOR_RADIAN_RANGE_BETWEEN_CRITICAL_LEVELS_TO_WAIT_FOR_CLAW);
        }

        /**
         * @return The radian of the elevator at the upper bound of the critical level. See {@link #ELEVATOR_RADIAN_RANGE_BETWEEN_CRITICAL_LEVELS_TO_WAIT_FOR_CLAW}
         */
        private Angle getUpperElevatorRadian() {
            return elevatorAngle.plus(ELEVATOR_RADIAN_RANGE_BETWEEN_CRITICAL_LEVELS_TO_WAIT_FOR_CLAW);
        }

        /**
         * @return The radian of the elevator that is closest to the current position.
         * - If the current position is less than the elevator angle, return the lower elevator radian.
         * - If the current position is greater than the elevator angle, return the upper elevator radian.
         */
        public Angle getFirstElevatorRadian(Angle currentPosition) {
            return currentPosition.lt(elevatorAngle) ? getLowerElevatorRadian() : getUpperElevatorRadian();
        }

        /**
         * @return The radian of the elevator that is further from the current position.
         * - If the current position is less than the elevator angle, return the upper elevator radian.
         * - If the current position is greater than the elevator angle, return the lower elevator radian.
         */
        public Angle getSecondElevatorRadian(Angle currentPosition) {
            return currentPosition.lt(elevatorAngle) ? getUpperElevatorRadian() : getLowerElevatorRadian();
        }

        public CriticalLevel(Angle elevatorRadian, Rotation2d clawAngle) {
            this.elevatorAngle = elevatorRadian;
            this.clawAngle = clawAngle;
        }
    }
}
