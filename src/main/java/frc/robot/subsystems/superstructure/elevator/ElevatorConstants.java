package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.util.TunableValue;

/**
 * Constants for the elevator.
 */
public final class ElevatorConstants {

    private ElevatorConstants() {}

    /**
     * Position constants for the elevator.
     */
    public static final class Position {

        private Position() {}

        /**
         * The initial height of the claw.
         */
        public static final Distance INITIAL_HEIGHT_CLAW = Meters.of(0.5);

        public static final Distance MIN_HEIGHT = Meters.of(0);
        public static final Distance MAX_HEIGHT = Meters.of(2.0);

        public static final Distance STAGE_1_HEIGHT = Meters.of(0.7);
        public static final Distance STAGE_1_MAX_HEIGHT = Meters.of(1.0);
        public static final Distance STAGE_2_HEIGHT = Meters.of(0.5);
    }

    /**
     * @return The radian position of the elevator motor from the height.
     * @param height - The height of the elevator.
     */
    public static Angle getMotorPositionFromHeight(Distance height) {
        return Radians.of(height.in(Meters) / ELEVATOR_RADIANS_TO_METERS);
    }

    /**
     * @return The height of the elevator from the radian position of the motor.
     * @param position - The position of the motor.
     */
    public static Distance getHeightFromMotorPosition(Angle position) {
        return Meters.of(position.in(Radians) * ELEVATOR_RADIANS_TO_METERS);
    }

    /**
     * The tolerance for the elevator to be considered at the target position.
     */
    public static final Distance ELEVATOR_DISTANCE_TOLERANCE = Meters.of(0.05);

    public static final Angle ELEVATOR_RAD_TOLERANCE = Radians.of(0.2);
    public static final Angle ELEVATOR_RAD_TOLERANCE_NOT_NEARER = Radians.of(1.5);

    /**
     * The CAN ID of the elevator motor.
     */
    public static final int ELEVATOR_MOTOR_ID = 15;
    public static final int ELEVATOR_FOLLOWER_MOTOR_ID = 20;

    public static final double ELEVATOR_MAX_OUTPUT = 0.875;
    public static final double ELEVATOR_TOP_INPUT = 0.55;
    public static final Angle ELEVATOR_TOP_THRESHOLD = Radians.of(20);

    public static final double ELEVATOR_GEAR_RATIO = 48.0;

    public static final Angle ELEVATOR_MIN_POSITION = Radians.of(-2);
    public static final Angle ELEVATOR_MAX_POSITION = Radians.of(30);

    public static final boolean ELEVATOR_MOTOR_INVERTED = false;
    public static final Current ELEVATOR_MOTOR_CURRENT_LIMIT = Amps.of(40);

    /** Rotations to radians */
    public static final double ELEVATOR_MOTOR_POSITION_FACTOR = (2 * Math.PI) / ELEVATOR_GEAR_RATIO;
    public static final Distance ELEVATOR_DRUM_RADIUS = Inches.of(0.75);

    /** Radians to meters */
    public static final double ELEVATOR_RADIANS_TO_METERS = ((16 - 1.75) / 8.6) * Inches.of(1).in(Meters);
    public static final AngularVelocity ELEVATOR_MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(10);
    public static final AngularAcceleration ELEVATOR_MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(20);
    public static final double AMOUNT_PER_FRAME = 0.3;

    public static final LinearVelocity ELEVATOR_SIM_MAX_VELOCITY = MetersPerSecond.of(1);
    public static final LinearAcceleration ELEVATOR_SIM_MAX_ACCELERATION = MetersPerSecondPerSecond.of(0.65);

    public static final TrapezoidProfile.Constraints PROFILED_ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(
        ELEVATOR_MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
        ELEVATOR_MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond)
    );

    // PID constants
    // public static final double ELEVATOR_KP = 0.1;
    // public static final double ELEVATOR_KI = 0.0;
    // public static final double ELEVATOR_KD = 0.0;

    public static final TunableValue ELEVATOR_KP = new TunableValue("Elevator/KP", 0.7);
    public static final TunableValue ELEVATOR_KI = new TunableValue("Elevator/KI", 0.0);
    public static final TunableValue ELEVATOR_KD = new TunableValue("Elevator/KD", 0.0);

    // TODO: Tune these values
    public static final Mass ELEVATOR_MASS = Kilograms.of(5.0);

    // Simulation constants
    public static final DCMotor SIM_MOTOR = DCMotor.getNEO(2);
    public static final MomentOfInertia SIM_MOI = KilogramSquareMeters.of(0.1);

    public static final double SIM_KP = 7.0;
    public static final double SIM_KI = 0.0;
    public static final double SIM_KD = 0.0;
    public static final double SIM_KF = 1.0;

    public static final double SIM_KS = 0.0; // volts (V)
    public static final double SIM_KG = 0.762; // volts (V)
    public static final double SIM_KV = 0.762; // volt per velocity (V/(m/s))
    public static final double SIM_KA = 0.0; // volt per acceleration (V/(m/s²))
}
