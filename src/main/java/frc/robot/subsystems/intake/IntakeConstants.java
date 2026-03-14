package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.util.WrappedSpark;
import java.util.function.Function;
import org.ironmaple.simulation.IntakeSimulation;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
import yams.mechanisms.config.ArmConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

/**
 * Constants for the Intake subsystem.
 */
public final class IntakeConstants {

    private IntakeConstants() {}

    public static final double INTAKE_RUN_SPEED = 0.95;

    public static final SmartMotorControllerConfig rollerMotorConfig = WrappedSpark.createCustomSparkMaxConfig()
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(3)
        // Motor properties to prevent over currenting.
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(43))
        .withVoltageCompensation(Volts.of(11.5))
        .withClosedLoopRampRate(Seconds.of(0.2))
        .withOpenLoopRampRate(Seconds.of(0.2));

    // sim
    public static final Distance SIM_INTAKE_LENGTH = Inches.of(5);
    public static final Mass SIM_INTAKE_MASS = Pounds.of(15);

    // Intake positions
    public static final Angle INTAKE_RETRACTED_ANGLE = Degrees.of(70);
    public static final Angle INTAKE_RETRACTED_ANGLE_SETPOINT = INTAKE_RETRACTED_ANGLE.minus(Degrees.of(15));
    public static final Angle INTAKE_DEPLOYED_ANGLE = Degrees.of(170);

    // public static final TrapezoidProfile.State retractGoalState = new TrapezoidProfile.State(
    //     INTAKE_RETRACTED_ANGLE.in(Rotations),
    //     0
    // );
    // public static final TrapezoidProfile.State deployGoalState = new TrapezoidProfile.State(
    //     INTAKE_DEPLOYED_ANGLE.in(Rotations),
    //     0
    // );

    public static final SmartMotorControllerConfig deployMotorConfig = WrappedSpark.createCustomSparkMaxConfig()
        .withSubsystem(new Subsystem() {})
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing((5 * 5 * 22.0) / 15.0))
        // 15:22
        // Feedback Constants (PID Constants)
        .withClosedLoopController(12, 0.0, 0.0, DegreesPerSecond.of(375), DegreesPerSecondPerSecond.of(440))
        .withSimClosedLoopController(7, 0.0, 0.0, DegreesPerSecond.of(350), DegreesPerSecondPerSecond.of(425))
        .withClosedLoopTolerance(Degrees.of(1))
        // Feedforward Constants
        // See https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control#manually-finding-kcos-and-ks-for-an-arm
        // V1 = 0.76
        // V2 = 0.6
        // .withFeedforward(new ArmFeedforward(0.08, 0.68, 0.0, 0.0))
        .withFeedforward(new ArmFeedforward(0.08, 0.68, 1.2022 * 2 * Math.PI, 0.38596 * 2 * Math.PI))
        .withSimFeedforward(new ArmFeedforward(0.078431, 0.46875, 0.68997 * 2 * Math.PI, 0.025313 * 2 * Math.PI))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(37))
        .withClosedLoopRampRate(Seconds.of(0.1))
        .withOpenLoopRampRate(Seconds.of(0.1));

    public static final Angle DEPLOY_TARGET_TOLERANCE = Degrees.of(6.7);
    public static final Angle RETRACT_TARGET_TOLERANCE = Degrees.of(10);
    public static final Angle INTAKE_SLIGHTLY_ABOVE_DEPLOYED_ANGLE = INTAKE_DEPLOYED_ANGLE.minus(
        DEPLOY_TARGET_TOLERANCE
    ).plus(Degrees.of(0.05));

    public static final Function<SmartMotorController, ArmConfig> deployArmConfigFunction =
        (SmartMotorController deployMotor) ->
            new ArmConfig(deployMotor)
                .withLength(SIM_INTAKE_LENGTH)
                .withMass(SIM_INTAKE_MASS)
                .withHardLimit(INTAKE_RETRACTED_ANGLE, INTAKE_DEPLOYED_ANGLE)
                // TODO: limits
                .withSoftLimits(
                    INTAKE_RETRACTED_ANGLE.minus(Degrees.of(60)),
                    INTAKE_DEPLOYED_ANGLE.plus(Degrees.of(60))
                )
                .withStartingPosition(INTAKE_RETRACTED_ANGLE);

    // Sysid constants
    public static final Voltage SYSID_MAX_VOLTAGE = Volts.of(2.5);
    public static final Velocity<VoltageUnit> SYSID_RAMP_RATE = Volts.of(0.25).per(Seconds);
    public static final Time SYSID_TEST_DURATION = Seconds.of(10.0);

    // Auto intake (constants for swerve)
    public static final Angle MAX_AUTO_INTAKE_YAW_ASSIST = Degrees.of(15);
    public static final Distance YAW_ASSIST_ACTIVATION_DISTANCE = Inches.of(24);
    public static final LinearVelocity AUTO_INTAKE_APPROACH_VELOCITY = MetersPerSecond.of(2);

    // Simulation constants
    public static final IntakeSimulation.IntakeSide ORIENTATION = IntakeSimulation.IntakeSide.FRONT;
    public static final Rotation2d ORIENTATION_AS_ROTATION = Rotation2d.kZero;
    public static final Time SIMULATION_TIME_FOR_INTAKE_DEPLOY = Seconds.of(0.5);

    public static final Distance WIDTH = Meters.of(0.52);
    public static final Distance HALF_OF_WIDTH = WIDTH.div(2);

    public static final Distance LENGTH = Meters.of(0.2);
    public static final Distance INTAKE_FORWARD_OFFSET = Meters.of(0.430467);
    public static final Transform2d ROBOT_CENTER_TO_INTAKE_CENTER = new Transform2d(
        new Translation2d(INTAKE_FORWARD_OFFSET, Inches.of(0)),
        ORIENTATION_AS_ROTATION
    );

    public static final Translation3d CENTER_TO_INTAKE_PIVOT = new Translation3d(0.316696, 0, 0.241539);

    public static final int MAX_CAPACITY = 48;

    // 2d arrangement of fuel in intake visualization
    public static final int ROWS_OF_FUEL = 5;
    public static final int COLUMNS_OF_FUEL = 5;
}
