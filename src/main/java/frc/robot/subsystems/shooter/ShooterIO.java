package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {

        // Shoot motor data
        public SparkMotorControllerData leaderShootMotorData = new SparkMotorControllerData();
        public boolean leaderShootMotorConnected = true;

        public SparkMotorControllerData followerShootMotorData = new SparkMotorControllerData();
        public boolean followerShootMotorConnected = true;

        // Setpoint info
        public MutAngularVelocity shootSetpoint = RadiansPerSecond.mutable(0.0);
        public MutAngularVelocity shootSetpointProfiled = RadiansPerSecond.mutable(0.0);
        public MutAngularAcceleration shootSetpointAcceleration = RadiansPerSecondPerSecond.mutable(0.0);

        // Indexer motor data
        public SparkMotorControllerData indexerMotorData = new SparkMotorControllerData();
        public boolean indexerMotorConnected = true;

        public MutAngularVelocity indexerSetpointProfiled = RadiansPerSecond.mutable(0.0);
        public MutAngularAcceleration indexerSetpointAcceleration = RadiansPerSecondPerSecond.mutable(0.0);

        // Beam breaker data
        public boolean beamBreakerConnected = true;
        public boolean isFuelDetectedAtTopOfIndexer = false;
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
     * Runs the shooter with a duty cycle output.
     * @param dutyCycleOutput - The duty cycle output in voltage.
     */
    public default void runShooterDutyCycle(Voltage dutyCycleOutput) {}

    /**
     * Runs the indexer with a duty cycle output.
     * @param dutyCycleOutput - The duty cycle output in voltage.
     */
    public default void runIndexerDutyCycle(Voltage dutyCycleOutput) {}

    /**
     * Stops the indexer.
     */
    public default void stopIndexer() {}

    /**
     * Sets the target velocity for the indexer.
     * @param velocity - The target velocity in radians per second.
     */
    public default void setIndexerVelocitySetpoint(AngularVelocity velocity) {}

    /**
     * Called periodically during simulation to update the simulated mechanism's state.
     */
    public default void simIterate() {}
}
