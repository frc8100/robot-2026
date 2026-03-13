package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Gs;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.util.InvertibleInterpolatingDoubleTreeMap;
import frc.util.WrappedSpark;
import java.util.function.Function;
import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

/**
 * Constants for the Shooter subsystem.
 */
public final class ShooterConstants {

    private ShooterConstants() {}

    /**
     * The rotation offset to apply to the shooter's aiming calculations.
     * Ex. if the shooter shoots fuel out the back of the robot, this would be 180 degrees.
     */
    public static final Rotation2d AIM_ROTATION_OFFSET = Rotation2d.kZero;

    /**
     * When calculating the future robot pose for aim to target feedforward, how far ahead to calculate.
     */
    public static final Time LOOKAHEAD_CALCULATION_TIME = Seconds.of(Constants.LOOP_PERIOD_SECONDS);

    /**
     * When predicting fuel trajectories, how far into the future to predict the final position of the fuel (for checking if it would score in the hub).
     */
    public static final Time PREDICT_FUEL_POSITION_LOOKAHEAD_TIME = Seconds.of(0.05);

    /**
     * The angle at which the shooter exits the fuel.
     */
    public static final Rotation2d exitAngle = Rotation2d.fromDegrees(53.0);

    /**
     * The acceleration due to gravity to use in fuel trajectory calculations.
     */
    public static final LinearAcceleration g = Gs.of(1);

    /**
     * The duty cycle to run the indexer at when feeding fuel into the shooter between [0, 1].
     */
    public static final double INDEXER_OUTPUT = 0.5;

    public static final AngularVelocity INDEXER_SPEED = RadiansPerSecond.of(175);

    public static final AngularVelocity SHOOTER_MIN_SPEED = RPM.of(1000);
    public static final AngularVelocity SHOOTER_MAX_SPEED = RadiansPerSecond.of(300);

    public static final Distance WHEEL_DIAMETER = Inches.of(4);

    // SOTM constants
    public static final Distance SOTM_MIN_DISTANCE = Inches.of(12);
    public static final Distance SOTM_MAX_DISTANCE = Inches.of(200);
    public static final Distance SOTM_DISTANCE_STEP = Inches.of(2);

    // Sysid config
    public static final Voltage SHOOTER_SYSID_MAX_VOLTAGE = Volts.of(11.0);
    public static final Velocity<VoltageUnit> SHOOTER_SYSID_RAMP_RATE = Volts.of(1.0).per(Second);
    public static final Time SHOOTER_SYSID_TEST_DURATION = Seconds.of(11.0);

    // Motor configs
    public static final SmartMotorControllerConfig shootMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(
            0.003,
            0.0,
            0.0,
            RadiansPerSecondPerSecond.of(750),
            RadiansPerSecondPerSecond.per(Second).of(7500)
        )
        .withSimClosedLoopController(
            0.05,
            0.0,
            0.0,
            RadiansPerSecondPerSecond.of(750),
            RadiansPerSecondPerSecond.per(Second).of(7500)
        )
        // Feedforward Constants
        // .withFeedforward(new SimpleMotorFeedforward(0.1155, 0.039507 * 2 * Math.PI, 0.0045979 * 2 * Math.PI))
        .withFeedforward(new SimpleMotorFeedforward(0.0, 0.039507 * 2 * Math.PI, 0.0045979 * 2 * Math.PI))
        .withSimFeedforward(new SimpleMotorFeedforward(0.086627, 0.020285 * 2 * Math.PI, 0.003377 * 2 * Math.PI))
        .withGearing(2)
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withVoltageCompensation(Volts.of(12.0))
        .withStatorCurrentLimit(Amps.of(45))
        .withClosedLoopRampRate(Seconds.of(0.1))
        .withOpenLoopRampRate(Seconds.of(0.2));

    public static final SmartMotorControllerConfig indexerMotorConfig = WrappedSpark.createCustomSparkMaxConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(
            0.003,
            0.0,
            0.0,
            RadiansPerSecondPerSecond.of(200),
            RadiansPerSecondPerSecond.per(Second).of(400)
        )
        .withSimClosedLoopController(
            0.1,
            0.0,
            0.0,
            RadiansPerSecondPerSecond.of(200),
            RadiansPerSecondPerSecond.per(Second).of(400)
        )
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0.36775, 0.021249 * 2 * Math.PI, 0.0017431 * 2 * Math.PI))
        .withSimFeedforward(new SimpleMotorFeedforward(0.0, 0.059281 * 2 * Math.PI, 0.0046219 * 2 * Math.PI))
        .withGearing(1)
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(35))
        .withOpenLoopRampRate(Seconds.of(0.2))
        .withMomentOfInertia(
            KilogramSquareMeters.of(
                SingleJointedArmSim.estimateMOI(WHEEL_DIAMETER.in(Meters), Pounds.of(2).in(Kilograms))
            )
        );

    public static final Function<SmartMotorController, FlyWheelConfig> shooterConfig =
        (SmartMotorController shootMotor) ->
            new FlyWheelConfig(shootMotor)
                // Diameter of the flywheel.
                .withDiameter(WHEEL_DIAMETER)
                // Mass of the flywheel.
                .withMass(Pounds.of(2))
                // Maximum speed of the shooter.
                .withUpperSoftLimit(SHOOTER_MAX_SPEED);

    // Simulation constants
    public static final Transform3d transformFromRobotCenter = new Transform3d(
        new Translation3d(Inches.of(-6.5), Inches.of(0.0), Inches.of(28.0)),
        new Rotation3d(AIM_ROTATION_OFFSET)
    );

    public static final Transform3d leftFuelStreamTransform = new Transform3d(
        new Translation3d(Inches.of(0), Inches.of(5), Inches.of(6)),
        Rotation3d.kZero
    );

    public static final Transform3d rightFuelStreamTransform = new Transform3d(
        new Translation3d(Inches.of(0), Inches.of(-5), Inches.of(6)),
        Rotation3d.kZero
    );

    public static final Translation2d positionFromRobotCenter2d = transformFromRobotCenter
        .getTranslation()
        .toTranslation2d();
    public static final Transform2d transformFromRobotCenter2d = new Transform2d(
        positionFromRobotCenter2d,
        transformFromRobotCenter.getRotation().toRotation2d()
    );

    public static final AngularVelocity SIMULATION_INDEXER_VELOCITY_AT_MAX_OUTPUT = INDEXER_SPEED;
    public static final double SIMULATION_MAX_FUEL_PER_SECOND = 8.5 / 2.0;

    // Characterization constants
    /**
     * A map of distance to target in meters to required shooter motor velocity in radians per second.
     */
    public static final InvertibleInterpolatingDoubleTreeMap distanceToMotorAngularVelocityMap =
        new InvertibleInterpolatingDoubleTreeMap();

    /**
     * Data for populating the shooter distance to velocity map.
     * - First column: distance to target in meters
     * - Second column: required shooter velocity in radians per second
     */
    // TODO: fill in with real data
    private static final double[] shooterMapData = {
        // @formatter:off
        0.0, 0.0,
        1.0, 50,
        2.0, 120,
        3.0, 200,
        4.0, 300,
        5.0, 450,
        6.0, 600,
        // @formatter:on
    };

    static {
        for (int i = 0; i < shooterMapData.length; i += 2) {
            distanceToMotorAngularVelocityMap.put(shooterMapData[i], shooterMapData[i + 1]);
        }
    }
}
