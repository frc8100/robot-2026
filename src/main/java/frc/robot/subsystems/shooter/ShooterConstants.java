package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
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

    public static final Angle exitAngle = Degrees.of(60.0);

    public static final LinearAcceleration g = MetersPerSecondPerSecond.of(9.81);

    public static final SmartMotorControllerConfig shootMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withGearing(36)
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(30))
        .withClosedLoopRampRate(Seconds.of(0.2))
        .withOpenLoopRampRate(Seconds.of(0.2));

    public static final Function<SmartMotorController, FlyWheelConfig> shooterConfig = (SmartMotorController motor) ->
        new FlyWheelConfig(motor)
            // Diameter of the flywheel.
            .withDiameter(Inches.of(4))
            // Mass of the flywheel.
            .withMass(Pounds.of(1))
            // Maximum speed of the shooter.
            .withUpperSoftLimit(RPM.of(1000));

    // Simulation constants
    public static final Transform3d positionFromRobotCenter = new Transform3d(
        new Translation3d(Inches.of(12.0), Inches.of(0.0), Inches.of(6)),
        Rotation3d.kZero
    );
    public static final Translation2d positionFromRobotCenter2d = positionFromRobotCenter
        .getTranslation()
        .toTranslation2d();
}
