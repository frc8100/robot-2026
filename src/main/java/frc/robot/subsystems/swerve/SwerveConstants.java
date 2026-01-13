package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import frc.util.SwerveFeedForwards;
import frc.util.TunableValue;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * Contains constants and configurations for the swerve drive (for CAN IDs, see {@link frc.robot.subsystems.swerve.SwerveModuleSpecificConstants})
 */
public class SwerveConstants {

    private SwerveConstants() {}

    public static final int NUMBER_OF_SWERVE_MODULES = 4;

    /**
     * How often the odometry is updated (in Hz). This is independent of the robot's.
     */
    public static final double ODOMETRY_FREQUENCY_HZ = 100;

    /**
     * How often the status signals for CANCoders and the Pigeon 2 is updated (in Hz).
     */
    public static final double STATUS_SIGNAL_FREQUENCY_HZ = 50;

    /**
     * The deadband for the sticks. This is the range of values that will be considered 0.
     */
    public static final double DRIVE_STICK_DEADBAND = 0.1;

    /**
     * When in semi-auto mode, the automatically calculated ChassisSpeeds will be adjusted by the controller inputs multiplied by this value.
     */
    public static final double NUDGE_TRANSLATION_INPUT_MULTIPLIER = 0.5;
    public static final double NUDGE_ROTATION_INPUT_MULTIPLIER = 0.5;

    // Percent output value limit for angle and drive motors
    public static final double MAX_DRIVE_POWER = 0.925;
    public static final double MAX_ANGLE_POWER = 0.9;

    // Always ensure Gyro is CCW+ CW-
    public static final boolean IS_GYRO_INVERTED = false;

    // Drivetrain constants
    /**
     * The distance between the two front modules (between FL-FR or BL-BR).
     */
    public static final Distance FRONT_MODULE_BASE = Inches.of(22.75);

    /**
     * The distance between the two side modules (between FL-BL or FR-BR).
     */
    public static final Distance SIDE_MODULE_BASE = Inches.of(22.75);

    /**
     * The radius from the center of the robot to any of the swerve modules.
     */
    public static final Distance DRIVE_BASE_RADIUS = Meters.of(
        Math.hypot(FRONT_MODULE_BASE.in(Meters) / 2.0, SIDE_MODULE_BASE.in(Meters) / 2.0)
    );

    public static final Distance FRONT_FRAME_LENGTH = Inches.of(28);
    public static final Distance SIDE_FRAME_LENGTH = Inches.of(28);

    // TODO: measure this
    public static final Distance FRONT_BUMPER_LENGTH = Inches.of(32);
    public static final Distance SIDE_BUMPER_LENGTH = Inches.of(32);

    public static final Distance WHEEL_RADIUS = Inches.of(2.0);
    public static final Distance WHEEL_CIRCUMFERENCE = WHEEL_RADIUS.times(2 * Math.PI);

    /**
     * The locations of the swerve modules relative to the center of the robot. Used in {@link SwerveDriveKinematics}
     */
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
        new Translation2d(SIDE_MODULE_BASE.div(2), FRONT_MODULE_BASE.div(2)),
        new Translation2d(SIDE_MODULE_BASE.div(2), FRONT_MODULE_BASE.div(-2)),
        new Translation2d(SIDE_MODULE_BASE.div(-2), FRONT_MODULE_BASE.div(2)),
        new Translation2d(SIDE_MODULE_BASE.div(-2), FRONT_MODULE_BASE.div(-2)),
    };

    // Standard deviations for the PoseEstimator
    public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

    // SDS MK4i with L2 drive gearing
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double ANGLE_GEAR_RATIO = ((150.0 / 7.0) / 1.0);

    // Motor inverts
    public static final boolean IS_ANGLE_MOTOR_INVERTED = true;
    public static final boolean IS_DRIVE_MOTOR_INVERTED = false;
    public static final boolean IS_CANCODER_INVERTED = false;

    // Current limiting
    public static final Current ANGLE_CONTINUOUS_CURRENT_LIMIT = Amps.of(22.5);
    // TODO: tune these values (https://docs.revrobotics.com/brushless/home/faq#neo-v1.1)
    public static final Current DRIVE_CONTINUOUS_CURRENT_LIMIT = Amps.of(40);

    /**
     * The time to wait after the robot is still before syncing the swerve module encoders.
     */
    public static final Time TIME_AFTER_STILL_SYNC_ENCODERS = Seconds.of(0.35);

    /**
     * The speed below which the robot is considered "still" (in m/s).
     */
    public static final double STILL_MPS = 0.075;

    // TODO: Change these to UPPER_SNAKE_CASE

    // Angle Motor PID Values
    // TODO: tune
    public static final double angleKP = 10.0;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;

    public static final TunableValue angleKPTunable = new TunableValue("Drive/AngleKP", angleKP);
    public static final TunableValue angleKDTunable = new TunableValue("Drive/AngleKD", angleKD);

    public static final double angleSimKP = 20.0;
    public static final double angleSimKD = 0.1;

    // Drive Motor PID Values
    public static final double driveKP = 0.01;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.005;

    public static final TunableValue driveKPTunable = new TunableValue("Drive/kP", driveKP);
    public static final TunableValue driveKDTunable = new TunableValue("Drive/kD", driveKD);

    public static final double driveSimKP = 0.2;
    public static final double driveSimKD = 0.0;

    /**
     * See {@link frc.util.SwerveFeedForwards} for feedforward values.
     */

    // Swerve path constraints
    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.75);
    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(11.5);

    public static final VelocityUnit<LinearAccelerationUnit> MetersPerSecond3 = MetersPerSecondPerSecond.per(Second);
    public static final Measure<VelocityUnit<LinearAccelerationUnit>> MAX_JERK = MetersPerSecond3.of(10);

    // ! IMPORTANT: The actual max angular velocity is much higher
    public static final AngularVelocity MAX_ANGULAR_VELOCITY_OF_ROBOT = RadiansPerSecond.of(7.0);

    public static final AngularVelocity MAX_ANGULAR_VELOCITY_OF_SWERVE_MODULE = RadiansPerSecond.of(7.5);

    /**
     * The angular velocity used during teleop driving.
     * When the controller axis (for rotation) is at maximum, this angular velocity will be used.
     */
    public static final AngularVelocity ANGULAR_VELOCITY_FOR_TELEOP = RotationsPerSecond.of(1);

    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(10.0);

    public static final APConstraints AUTO_PILOT_CONSTRAINTS = new APConstraints(
        MAX_SPEED.in(MetersPerSecond),
        MAX_ACCELERATION.in(MetersPerSecondPerSecond),
        MAX_JERK.in(MetersPerSecond3)
    );

    // Tolerances for being "at" the target pose
    public static final Distance positionTolerance = Inches.of(1.25);
    public static final Distance beelineRadius = Inches.of(6);
    public static final Angle angleTolerance = Degrees.of(3);
    public static final LinearVelocity speedTolerance = InchesPerSecond.of(4);
    public static final LinearVelocity targetVelocity = MetersPerSecond.of(0);

    public static final APProfile autopilotProfile = new APProfile(AUTO_PILOT_CONSTRAINTS)
        .withErrorXY(positionTolerance)
        .withErrorTheta(angleTolerance)
        .withBeelineRadius(beelineRadius);

    // Auto aim config
    public static final AngularVelocity MAX_AUTO_AIM_ROBOT_ANGULAR_VELOCITY = RadiansPerSecond.of(6.0);
    // TODO: Maybe scale this based off distance (further = less tolerance)
    public static final Angle AUTO_AIM_ANGLE_TOLERANCE = Degrees.of(4.0);

    // Tipping config
    /**
     * Whether the output from anti-tipping is enabled.
     */
    // TODO: Test
    public static final boolean IS_ANTI_TIPPING_ENABLED = false;

    /**
     * Whether the gyro should record pitch and roll, and calculate tipping state.
     * If anti-tipping is disabled and this is enabled, information will be recorded but not used.
     */
    public static final boolean IS_GYRO_RECORD_PITCH_ROLL_TIPPING_STATE = IS_ANTI_TIPPING_ENABLED;

    // m/s per degree of pitch/roll
    public static final double ANTI_TIPPING_KP = 0.04;
    public static final Angle TIPPING_THRESHOLD = Degrees.of(3.0);
    public static final LinearVelocity MAX_ANTI_TIP_VELOCITY = MetersPerSecond.of(2.75);

    // Path Planner Values
    public static final Mass ROBOT_MASS = Pounds.of(110);
    public static final double WHEEL_COF = 1.2;
    // TODO: measure this
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(3.9506340342);

    public static final Pose2d initialPose = new Pose2d(7.576, 2.29, Rotation2d.fromDegrees(180));
    public static final PathConstraints pathConstraints = new PathConstraints(
        MAX_SPEED,
        MAX_ACCELERATION,
        MAX_ANGULAR_VELOCITY_OF_ROBOT,
        MAX_ANGULAR_ACCELERATION
    );

    public static final PIDConstants PP_INITIAL_TRANSLATION_PID = new PIDConstants(4.5, 0.1);
    public static final PIDConstants PP_ENDING_TRANSLATION_PID = new PIDConstants(4.67, 0.075);
    public static final PIDConstants PP_ROTATION_PID = new PIDConstants(4.0, 0.02);

    public static final PPHolonomicDriveController PP_INITIAL_PID_CONTROLLER = new PPHolonomicDriveController(
        SwerveConstants.PP_INITIAL_TRANSLATION_PID,
        SwerveConstants.PP_ROTATION_PID
    );

    // Simulator DC Motors
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    public static final RobotConfig pathPlannerConfig = new RobotConfig(
        ROBOT_MASS,
        ROBOT_MOI,
        new ModuleConfig(
            WHEEL_RADIUS,
            MAX_SPEED,
            WHEEL_COF,
            driveGearbox,
            DRIVE_GEAR_RATIO,
            DRIVE_CONTINUOUS_CURRENT_LIMIT,
            NUMBER_OF_SWERVE_MODULES
        ),
        MODULE_TRANSLATIONS
    );

    /**
     * Maplesim configuration for the swerve drive.
     */
    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
        .withCustomModuleTranslations(MODULE_TRANSLATIONS)
        .withBumperSize(FRONT_BUMPER_LENGTH, SIDE_BUMPER_LENGTH)
        .withRobotMass(ROBOT_MASS)
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(
            new SwerveModuleSimulationConfig(
                driveGearbox,
                turnGearbox,
                DRIVE_GEAR_RATIO,
                ANGLE_GEAR_RATIO,
                Volts.of(0.1),
                Volts.of(0.1),
                WHEEL_RADIUS,
                KilogramSquareMeters.of(0.02),
                WHEEL_COF
            )
        );

    /**
     * @return The PathPlanner RobotConfig
     */
    public static RobotConfig getRobotConfig() {
        return pathPlannerConfig;
    }

    /**
     * @return The CANCoder configuration.
     */
    public static CANcoderConfiguration getCANcoderConfig() {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

        // Invert the sensor direction if necessary
        MagnetSensorConfigs magnetSenorConfig = new MagnetSensorConfigs()
            .withSensorDirection(
                IS_CANCODER_INVERTED
                    ? SensorDirectionValue.Clockwise_Positive
                    : SensorDirectionValue.CounterClockwise_Positive
            );

        // TODO: maybe apply angle offset here?

        canCoderConfig.withMagnetSensor(magnetSenorConfig);

        return canCoderConfig;
    }

    /**
     * @return The angle motor configuration.
     * Includes the relative encoder configuration.
     */
    public static SparkMaxConfig getAngleMotorConfig() {
        // Assign the relative angle encoder and configure it
        SparkMaxConfig angleConfig = new SparkMaxConfig();

        angleConfig
            .smartCurrentLimit((int) SwerveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT.in(Amps))
            .inverted(SwerveConstants.IS_ANGLE_MOTOR_INVERTED)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .openLoopRampRate(0.2);

        angleConfig.encoder
            // Rotations to degrees
            .positionConversionFactor((2 * Math.PI) / (ANGLE_GEAR_RATIO))
            // RPM to radians per second
            .velocityConversionFactor(((2 * Math.PI) / (ANGLE_GEAR_RATIO)) / 60.0);

        // Configure the PID controller for the angle motor
        angleConfig.closedLoop
            .pid(SwerveConstants.angleKP, SwerveConstants.angleKI, SwerveConstants.angleKD)
            .outputRange(-SwerveConstants.MAX_ANGLE_POWER, SwerveConstants.MAX_ANGLE_POWER)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI);

        angleConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / SwerveConstants.ODOMETRY_FREQUENCY_HZ))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        return angleConfig;
    }

    /**
     * @return The drive motor configuration.
     * Includes the relative encoder configuration.
     */
    public static SparkMaxConfig getDriveMotorConfig() {
        // Get the config for the encoders
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        driveConfig
            .smartCurrentLimit((int) SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT.in(Amps))
            .inverted(SwerveConstants.IS_DRIVE_MOTOR_INVERTED)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .openLoopRampRate(0.2);

        // Set the position and velocity conversion factors based on the SwerveConfig
        driveConfig.encoder
            // Rotations to wheel radians
            .positionConversionFactor((2 * Math.PI) / (DRIVE_GEAR_RATIO))
            // RPM to wheel radians per second
            .velocityConversionFactor(((2 * Math.PI) / (DRIVE_GEAR_RATIO)) / 60.0)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        // Configure the PID controller for the drive motor
        driveConfig.closedLoop
            .pid(SwerveConstants.driveKP, SwerveConstants.driveKI, SwerveConstants.driveKD)
            .outputRange(-SwerveConstants.MAX_DRIVE_POWER, SwerveConstants.MAX_DRIVE_POWER);

        driveConfig.closedLoop.feedForward
            .kS(SwerveFeedForwards.linearForceDriveFFConstantsReal.kS())
            .kV(SwerveFeedForwards.linearForceDriveFFConstantsReal.kV())
            .kA(SwerveFeedForwards.linearForceDriveFFConstantsReal.kA());

        driveConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / SwerveConstants.ODOMETRY_FREQUENCY_HZ))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        return driveConfig;
    }
}
