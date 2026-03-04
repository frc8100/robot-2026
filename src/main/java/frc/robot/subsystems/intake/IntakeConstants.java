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
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

    public static final double INTAKE_RUN_SPEED = 0.7;

    public static final SmartMotorControllerConfig intakeMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.OPEN_LOOP)
        .withGearing(3)
        // Motor properties to prevent over currenting.
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(32))
        .withClosedLoopRampRate(Seconds.of(0.2))
        .withOpenLoopRampRate(Seconds.of(0.35));

    // sim
    public static final Distance SIM_INTAKE_LENGTH = Inches.of(7);
    public static final Mass SIM_INTAKE_MASS = Pounds.of(8);

    // Intake positions
    public static final Angle INTAKE_RETRACTED_ANGLE = Degrees.of(85);
    public static final Angle INTAKE_DEPLOYED_ANGLE = Degrees.of(195);

    public static final SmartMotorControllerConfig deployMotorConfig = new SmartMotorControllerConfig()
        // .withMomentOfInertia(
        //     KilogramSquareMeters.of(
        //         SingleJointedArmSim.estimateMOI(SIM_INTAKE_LENGTH.in(Meters), SIM_INTAKE_MASS.in(Kilograms))
        //     ).times(0.1)
        // )
        .withSubsystem(new Subsystem() {})
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5), new Sprocket(15.0 / 22.0)))
        // 15:22
        // Feedback Constants (PID Constants)
        // .withClosedLoopController(50, 0.0, 0.0, DegreesPerSecond.of(260), DegreesPerSecondPerSecond.of(800))
        // .withSimClosedLoopController(13, 0.0, 0.0, DegreesPerSecond.of(7000), DegreesPerSecondPerSecond.of(1000))

        // MAXMotion sucks so don't use constraints
        .withClosedLoopController(2.0, 0.0, 0.0)
        .withSimClosedLoopController(2.0, 0.0, 0.0)
        // Feedforward Constants
        // See https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control#manually-finding-kcos-and-ks-for-an-arm
        // V1 = 0.76
        // V2 = 0.6
        // .withFeedforward(new ArmFeedforward(0.08, 0.68, 0.0, 0.0))
        .withFeedforward(new ArmFeedforward(0.0, 0.0, 0.0, 0.0))
        // .withSimFeedforward(new ArmFeedforward(0.048643, 0.80614, 0.72715, 0.023721))
        .withSimFeedforward(new ArmFeedforward(0.0, 0.0, 0.0, 0.0))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(37))
        .withClosedLoopRampRate(Seconds.of(0.1))
        .withOpenLoopRampRate(Seconds.of(0.2))
        .withStartingPosition(INTAKE_RETRACTED_ANGLE);

    public static final Angle DEPLOY_TARGET_TOLERANCE = Degrees.of(10);

    public static final Function<SmartMotorController, ArmConfig> deployArmConfigFunction =
        (SmartMotorController deployMotor) ->
            new ArmConfig(deployMotor)
                .withLength(SIM_INTAKE_LENGTH)
                .withMass(SIM_INTAKE_MASS)
                .withHardLimit(INTAKE_RETRACTED_ANGLE, INTAKE_DEPLOYED_ANGLE)
                // TODO: limits
                // .withSoftLimits(
                //     INTAKE_RETRACTED_ANGLE.minus(Degrees.of(10)),
                //     INTAKE_DEPLOYED_ANGLE.plus(Degrees.of(10))
                // )
                .withStartingPosition(INTAKE_RETRACTED_ANGLE.plus(Degrees.of(10)));

    // Sysid constants
    public static final Voltage SYSID_MAX_VOLTAGE = Volts.of(2.0);
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
