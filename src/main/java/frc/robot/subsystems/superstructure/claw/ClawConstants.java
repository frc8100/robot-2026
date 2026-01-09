package frc.robot.subsystems.superstructure.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.util.TunableValue;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

/**
 * Constants for the claw. Includes CAN Ids.
 */
public final class ClawConstants {

    private ClawConstants() {}

    /**
     * Contains the angle measures for certain claw rotations
     */
    public static final class RotationPositions {

        private RotationPositions() {}

        /**
         * The initial angle of the claw when it is resting (against the mechanical lock).
         * The lower it is, the more "horizontal" the claw is.
         */
        public static final Rotation2d CLAW_ANGLE_OFFSET = Rotation2d.fromDegrees(35);

        public static final Rotation2d RESTING_ANGLE = Rotation2d.fromDegrees(0);

        /**
         * Amount to rotate the claw to when moving the elevator up.
         */
        public static final Rotation2d CLAW_HOLDING_POSITION = Rotation2d.fromDegrees(110).minus(CLAW_ANGLE_OFFSET);

        public static final Rotation2d CLAW_L4_SCORING_POSITION = Rotation2d.fromDegrees(95).minus(CLAW_ANGLE_OFFSET);

        public static final Rotation2d CLAW_ALGAE_THROW_POSITION = Rotation2d.fromDegrees(75).minus(CLAW_ANGLE_OFFSET);

        /**
         * The horizontal, front/back translation of the claw from the origin of the second elevator stage.
         */
        public static final double ELEVATOR_TO_CLAW_X = 0.27;

        /**
         * The vertical, up/down translation of the claw from the origin of the second elevator stage.
         */
        public static final double ELEVATOR_TO_CLAW_Z = 0.37;

        /**
         * The translation from the origin of the claw to where the coral is stored.
         */
        private static final double CLAW_TO_CORAL = 0.3;

        /**
         * @return The translation (x only) from the origin of the claw to where the coral is stored.
         * @param angleRadians - The angle of the claw, in radians, with the offset applied.
         */
        public static Translation2d getClawToCoralX(double angleRadians) {
            // Undo the offset
            angleRadians = angleRadians + CLAW_ANGLE_OFFSET.getRadians();

            return new Translation2d(CLAW_TO_CORAL * Math.sin(angleRadians - (Math.PI / 4)), 0);
        }

        /**
         * @return The z (vertical) translation (in meters) from the origin of the claw to where the coral is stored.
         * @param angleRadians - The angle of the claw, in radians, with the offset applied.
         */
        public static double getClawToCoralZ(double angleRadians) {
            // Undo the offset
            angleRadians = angleRadians + CLAW_ANGLE_OFFSET.getRadians();

            return CLAW_TO_CORAL * Math.cos(angleRadians - (Math.PI / 4));
        }
    }

    /**
     * The direction of the intake/outtake.
     */
    public enum IntakeOuttakeDirection {
        /**
         * Runs the intake/outtake back in. This is also used to intake algae.
         */
        BACK(1),

        /**
         * Runs the intake/outtake out. This moves the coral from the ramp to the claw and is also the same direction as the outtake (move the coral out of the claw).
         */
        OUTTAKE(-1);

        /**
         * The direction of the intake/outtake.
         * If the direction is 1, the intake/outtake is in.
         * If the direction is -1, the intake/outtake is out.
         */
        private final int direction;

        /**
         * @return The direction of the intake/outtake.
         */
        public int getDirection() {
            return direction;
        }

        /**
         * Creates a new IntakeOuttakeDirection based on the given direction.
         */
        IntakeOuttakeDirection(int direction) {
            this.direction = direction;
        }
    }

    public static final Time INTAKE_TIME = Seconds.of(8);
    public static final Time OUTTAKE_TIME = Seconds.of(1.2);
    public static final Time ALGAE_TIMEOUT_TIME = Seconds.of(10);

    /**
     * How much the claw can be off from the desired angle before it is considered "in position".
     * This is in radians.
     */
    public static final Angle ANGLE_TOLERANCE_RADIANS = Degrees.of(7.5);

    /**
     * Deadband for the claw (controller input).
     */
    public static final double CONTROLLER_DEADBAND = 0.1;

    // Turn motor configs
    public static final int ANGLE_MOTOR_ID = 9;
    /**
     * The maximum power for the angle motor, from 0-1.
     */
    public static final double MAX_ANGLE_POWER = 0.275;

    public static final double ANGLE_GEAR_RATIO = 30;
    public static final boolean IS_ANGLE_MOTOR_INVERTED = true;
    public static final Current ANGLE_MOTOR_CURRENT_LIMIT = Amps.of(40);
    /** Rotations to radians */
    public static final double ANGLE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / ANGLE_GEAR_RATIO;

    // TODO:
    public static final Angle MAX_WHEN_AT_BOTTOM = Radians.of(2.698);
    public static final Angle MAX_CLAW_ANGLE = Radians.of(4);
    public static final Angle MIN_CLAW_ANGLE = Radians.of(-0.2);

    // unused
    // TODO: why is this not affected by conversion factor?
    public static final AngularVelocity MAX_ANGLE_SPEED = RadiansPerSecond.of(4);
    public static final AngularAcceleration MAX_ANGLE_ACCELERATION = RadiansPerSecondPerSecond.of(15);

    // PID configs
    public static final TunableValue ANGLE_KP = new TunableValue("Claw/KP", 0.7);
    public static final TunableValue ANGLE_KI = new TunableValue("Claw/KI", 0.0);
    public static final TunableValue ANGLE_KD = new TunableValue("Claw/KD", 0.1);

    public static final TunableValue PROFILED_ANGLE_KP = new TunableValue("Claw/P_KP", 0.7);
    public static final TunableValue PROFILED_ANGLE_KI = new TunableValue("Claw/P_KI", 0.0);
    public static final TunableValue PROFILED_ANGLE_KD = new TunableValue("Claw/P_KD", 0.1);
    public static final TrapezoidProfile.Constraints PROFILED_ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAX_ANGLE_SPEED.in(RadiansPerSecond),
        MAX_ANGLE_ACCELERATION.in(RadiansPerSecondPerSecond)
    );

    // Outtake motor configs
    public static final int OUTTAKE_MOTOR_ID = 6;
    public static final double OUTTAKE_GEAR_RATIO = 5;
    public static final boolean IS_OUTTAKE_MOTOR_INVERTED = false;
    public static final Current OUTTAKE_MOTOR_CURRENT_LIMIT = Amps.of(40);
    /** Rotations to radians */
    public static final double OUTTAKE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / OUTTAKE_GEAR_RATIO;

    /**
     * The percent power (from 0-1) to run the intake/outtake
     */
    public static final double OUTTAKE_MAX_OUTPUT = 0.15;

    // Simulator configs
    public static final DCMotor SIM_ANGLE_MOTOR = DCMotor.getNEO(1);
    public static final MomentOfInertia SIM_ANGLE_MOI = KilogramSquareMeters.of(0.1);
    public static final Voltage SIM_ANGLE_FRICTION_VOLTAGE = Volts.of(0.1);
    public static final double SIM_ANGLE_KP = 4.0;
    public static final double SIM_ANGLE_KI = 0.0;
    public static final double SIM_ANGLE_KD = 0.0;

    public static final SimMotorConfigs SIM_ANGLE_MOTOR_CONFIG = new SimMotorConfigs(
        SIM_ANGLE_MOTOR,
        ANGLE_GEAR_RATIO,
        SIM_ANGLE_MOI,
        SIM_ANGLE_FRICTION_VOLTAGE
    );

    public static final DCMotor SIM_OUTTAKE_MOTOR = DCMotor.getNEO(1);
    public static final AngularVelocity SIM_OUTTAKE_TARGET_VELOCITY = RadiansPerSecond.of(25);
    public static final MomentOfInertia SIM_OUTTAKE_MOI = KilogramSquareMeters.of(0.05);
    public static final Voltage SIM_OUTTAKE_FRICTION_VOLTAGE = Volts.of(0.1);
    public static final double SIM_OUTTAKE_KP = 0.6;
    public static final double SIM_OUTTAKE_KI = 0.0;
    public static final double SIM_OUTTAKE_KD = 0.0;

    public static final SimMotorConfigs SIM_OUTTAKE_MOTOR_CONFIG = new SimMotorConfigs(
        SIM_OUTTAKE_MOTOR,
        OUTTAKE_GEAR_RATIO,
        SIM_OUTTAKE_MOI,
        SIM_OUTTAKE_FRICTION_VOLTAGE
    );

    /**
     * The speed that the coral is ejected at.
     */
    public static final LinearVelocity SIM_OUTTAKE_EJECT_SPEED = MetersPerSecond.of(1.75);
}
