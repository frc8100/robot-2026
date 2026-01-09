package frc.robot.subsystems.superstructure.claw;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.util.GenericSparkIO;
import org.littletonrobotics.junction.AutoLog;

/**
 * The IO interface for the claw.
 */
public interface ClawIO extends GenericSparkIO<ClawIO.ClawIOInputs> {
    /**
     * The inputs that should be supplied by the IO interface.
     */
    @AutoLog
    public static class ClawIOInputs {

        /**
         * Whether coral is currently in the claw.
         * Note: This is currently unused (except in simulator), but should be used to determine
         * if the claw is holding a piece of coral using a sensor.
         */
        public boolean isCoralInClaw = false;

        // Inputs from the turn motor
        public boolean turnConnected = true;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnSupplyCurrentAmps = 0.0;
        public double turnTorqueCurrentAmps = 0.0;
        // public double turnTempCelsius = 0.0;
        public double turnSetpointRad = 0.0;

        public double turnTrapezoidalTargetVelocityRadPerSec = 0.0;
        public double turnTrapezoidalTargetPositionRad = 0.0;

        // Inputs from the outtake motor
        public boolean outtakeConnected = true;
        public double outtakePositionRad = 0.0;
        public double outtakeVelocityRadPerSec = 0.0;
        public double outtakeAppliedVolts = 0.0;
        public double outtakeSupplyCurrentAmps = 0.0;
        public double outtakeTorqueCurrentAmps = 0.0;
        // public double outtakeTempCelsius = 0.0;
        public double outtakeSetpointVelocityRadPerSec = 0.0;
    }

    /**
     * Stops both motors.
     */
    public default void stop() {}

    /**
     * Sets the desired turn position.
     * @param rotation - The desired turn position.
     */
    public default void setTurnPosition(Rotation2d rotation) {}

    /**
     * Increases the desired turn position.
     * @param rotation - The desired turn position.
     */
    public default void increaseTurnPosition(double valueToIncreaseBy) {}

    /**
     * Runs the outtake motor at the specified percent output.
     * @param motorInput - The percent output to run the motor at, from [-1, 1] without deadband.
     */
    public default void runOuttake(double motorInput) {}

    /**
     * Periodic function to be called by the subsystem.
     * Optional, do not update the IO here.
     */
    public default void periodic() {}

    /**
     * Resets the setpoint to the current position.
     */
    public default void resetSetpointToCurrentPosition() {}

    /**
     * Zeros the encoder to the given value.
     * @param value - The value to set the encoder to.
     */
    public default void zeroEncoder(double value) {}
}
