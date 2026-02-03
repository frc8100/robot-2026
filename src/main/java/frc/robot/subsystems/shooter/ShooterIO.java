package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {

        // Motor controller data for the intake motor
        public SparkMotorControllerData motorData = new SparkMotorControllerData();
        public boolean motorConnected = true;

        /**
         * The target exit velocity for the shooter in meters per second.
         */
        public MutLinearVelocity setpointExitLinearVelocity = MetersPerSecond.mutable(0.0);

        /**
         * The target exit velocity for the shooter in radians per second.
         */
        public MutAngularVelocity setpointExitAngularVelocity = RadiansPerSecond.mutable(0.0);
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    // test
    public default void testShoot() {}

    /**
     * Sets the target exit velocity for the shooter.
     * @param velocityMetersPerSecond - The target exit velocity in meters per second.
     */
    public default void setTargetExitVelocity(double velocityMetersPerSecond) {}
}
