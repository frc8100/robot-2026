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
        public SparkMotorControllerData shootMotorData = new SparkMotorControllerData();
        public boolean shootMotorConnected = true;

        public SparkMotorControllerData indexerMotorData = new SparkMotorControllerData();
        public boolean indexerMotorConnected = true;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    /**
     * Sets the target exit velocity for the shooter.
     * @param velocity - The target exit velocity in radians per second.
     */
    public default void setTargetShootMotorVelocity(AngularVelocity velocity) {}

    /**
     * Stops the shooter.
     */
    public default void stopShooter() {}

    /**
     * Runs the indexer.
     */
    public default void runIndexer() {}

    /**
     * Stops the indexer.
     */
    public default void stopIndexer() {}

    public default void simIterate() {}
}
