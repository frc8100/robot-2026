package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.Pair;
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
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(55))
            .withClosedLoopRampRate(Seconds.of(0.2))
            .withOpenLoopRampRate(Seconds.of(0.2))
            .withFollowers(Pair.of(spark, false));
}
