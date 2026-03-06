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

        // Motor controller data for the intake motor
        public SparkMotorControllerData shootMotorData = new SparkMotorControllerData();
        public boolean shootMotorConnected = true;

        public MutAngularVelocity shootSetpoint = RadiansPerSecond.mutable(0.0);
        public MutAngularVelocity shootSetpointProfiled = RadiansPerSecond.mutable(0.0);
        public MutAngularAcceleration shootSetpointAcceleration = RadiansPerSecondPerSecond.mutable(0.0);

        public SparkMotorControllerData indexerMotorData = new SparkMotorControllerData();
        public boolean indexerMotorConnected = true;

        public MutAngularVelocity indexerSetpointProfiled = RadiansPerSecond.mutable(0.0);
        public MutAngularAcceleration indexerSetpointAcceleration = RadiansPerSecondPerSecond.mutable(0.0);
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

    public default void runIndexerDutyCycle(Voltage dutyCycleOutput) {}

    public default void stopIndexer() {}

    public default void setIndexerVelocitySetpoint(AngularVelocity velocity) {}

    /**
     * Called periodically during simulation to update the simulated mechanism's state.
     */
    public default void simIterate() {}
}
