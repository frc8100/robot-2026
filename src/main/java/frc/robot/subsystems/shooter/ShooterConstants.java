package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Gs;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.util.InvertibleInterpolatingDoubleTreeMap;
import java.util.function.Function;
import org.ironmaple.simulation.IntakeSimulation;
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
    public static final Rotation2d AIM_ROTATION_OFFSET = Rotation2d.k180deg;

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
    public static final Rotation2d exitAngle = Rotation2d.fromDegrees(70.0);

    /**
     * The acceleration due to gravity to use in fuel trajectory calculations.
     */
    public static final LinearAcceleration g = Gs.of(1);

    /**
     * The duty cycle to run the indexer at when feeding fuel into the shooter between [0, 1].
     */
    public static final double INDEXER_OUTPUT = 0.5;

    // Sysid config
    public static final Voltage SHOOTER_SYSID_MAX_VOLTAGE = Volts.of(10.0);
    public static final Velocity<VoltageUnit> SHOOTER_SYSID_RAMP_RATE = Volts.of(1.0).per(Second);
    public static final Time SHOOTER_SYSID_TEST_DURATION = Seconds.of(10.0);

    // Motor configs
    public static final SmartMotorControllerConfig shootMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(0.1, 0.0, 0.0, RadiansPerSecond.of(90), RadiansPerSecondPerSecond.of(45))
        .withSimClosedLoopController(2, 0.0, 0.0, RadiansPerSecond.of(300), RadiansPerSecondPerSecond.of(600))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0.022028, 0.019888 * 2 * Math.PI, 0.0070252 * 2 * Math.PI))
        .withGearing(1)
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(50))
        .withClosedLoopRampRate(Seconds.of(0.2))
        .withOpenLoopRampRate(Seconds.of(0.2));

    public static final SmartMotorControllerConfig indexerMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(3)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(30))
        .withOpenLoopRampRate(Seconds.of(0.2));

    public static final Function<SmartMotorController, FlyWheelConfig> shooterConfig =
        (SmartMotorController shootMotor) ->
            new FlyWheelConfig(shootMotor)
                // Diameter of the flywheel.
                .withDiameter(Inches.of(4))
                // Mass of the flywheel.
                .withMass(Pounds.of(1))
                // Maximum speed of the shooter.
                .withUpperSoftLimit(RPM.of(1000));

    // Simulation constants
    // public static final Transform3d transformFromRobotCenter = new Transform3d(
    //     new Translation3d(Inches.of(12.0), Inches.of(0.0), Inches.of(6)),
    //     Rotation3d.kZero
    // );
    // TODO: test with real measurements
    public static final Transform3d transformFromRobotCenter = new Transform3d(
        new Translation3d(Inches.of(0), Inches.of(0), Inches.of(6)),
        new Rotation3d(AIM_ROTATION_OFFSET)
    );

    public static final Translation2d positionFromRobotCenter2d = transformFromRobotCenter
        .getTranslation()
        .toTranslation2d();
    public static final Transform2d transformFromRobotCenter2d = new Transform2d(
        positionFromRobotCenter2d,
        transformFromRobotCenter.getRotation().toRotation2d()
    );

    public static final AngularVelocity SIMULATION_INDEXER_VELOCITY_AT_MAX_OUTPUT = RadiansPerSecond.of(110);
    public static final double SIMULATION_MAX_FUEL_PER_SECOND = 5.5;

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
