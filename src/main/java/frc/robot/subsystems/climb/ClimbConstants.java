package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import java.util.function.Function;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

public class ClimbConstants {

    private ClimbConstants() {}

    public static final Function<SparkBase, SmartMotorControllerConfig> climbMotorConfig = spark ->
        new SmartMotorControllerConfig()
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withGearing(500)
            // Feedback Constants (PID Constants)
            .withClosedLoopController(6.0, 0.0, 0.0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(300))
            .withSimClosedLoopController(6.0, 0.0, 0.0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(300))
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(55))
            .withClosedLoopRampRate(Seconds.of(0.2))
            .withOpenLoopRampRate(Seconds.of(0.2))
            .withFollowers(Pair.of(spark, false));

    // Angle targets
    public static final Angle RETRACTED_ANGLE = Degrees.of(90.0);
    public static final Angle DEPLOY_ANGLE = Degrees.of(190.0);
    public static final Angle CLIMB_ANGLE = Degrees.of(50.0);

    public static final Angle TOLERANCE = Degrees.of(5.0);

    /**
     * The minimum current that the climb motors should draw when latched onto the bar. If the current is below this threshold, assume that the climb is not latched.
     */
    public static final Current MIN_LATCH_CURRENT = Amps.of(25.0);

    /**
     * The current that is drawn when the climb is at the target angle. If current is above this threshold, assume that the climb is at the target angle.
     */
    public static final Current MIN_AT_TARGET_CURRENT = Amps.of(45.0);

    /**
     * When climbing, this angle is the max that the climb can go back before it starts lifting the robot up.
     * Only used for visualization.
     */
    public static final Angle ANGLE_BEFORE_START_TO_LATCH = Degrees.of(120.0);

    /**
     * The position that the robot rotates around when climbing. This is used for visualization and simulation.
     */
    public static final Transform3d CLIMB_CENTER_OF_ROTATION = new Transform3d(
        new Translation3d(Inches.of(6), Inches.of(24), Inches.of(0)),
        Rotation3d.kZero
    );
}
