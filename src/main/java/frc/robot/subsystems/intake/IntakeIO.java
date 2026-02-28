package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.Voltage;
import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {

        // Deploy motor
        public SparkMotorControllerData deployMotorData = new SparkMotorControllerData();
        public boolean deployMotorConnected = true;

        // Intake roller motor
        public SparkMotorControllerData rollerMotorData = new SparkMotorControllerData();
        public boolean rollerMotorConnected = true;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /**
     * Runs the intake rollers at the given speed. Positive is intaking.
     * @param output - The duty cycle speed from [-1, 1].
     */
    public default void runRollerDutyCycle(double output) {}

    /**
     * Runs the deploy motor at the given duty cycle output. See {@link Intake.IntakeDeployDirection} for direction conventions.
     * @param output - The duty cycle output from [-1, 1].
     */
    public default void runDeployDutyCycle(double output) {}

    /**
     * Runs the deploy motor at the given voltage output. See {@link Intake.IntakeDeployDirection} for direction conventions.
     * @param output - The voltage output.
     */
    public default void runDeployVoltage(Voltage output) {}

    /**
     * Sets the deploy motor encoder position to the given position. This is used for resetting the encoder when we know the absolute position of the intake.
     * @param position - The position to set the deploy motor encoder to as a Rotation.
     */
    public default void setDeployEncoderPosition(Rotation2d position) {
        setDeployEncoderPosition(position.getMeasure());
    }

    /**
     * @param position - The position to set the deploy motor encoder to as a Measure.
     */
    public default void setDeployEncoderPosition(Angle position) {}

    /**
     * Sets the closed loop setpoint for the deploy motor.
     * @param setpoint - The setpoint to set for the deploy motor.
     */
    public default void setDeploySetpoint(Angle setpoint) {}

    /**
     * Set whether the soft limits (software on motor controller stops motor from moving past certain positions) are enabled on the deploy motor.
     * Used for testing. Also used for calibration sequence to return back to hard stop without worrying about soft limits.
     * Should be used with caution, as it can lead to mechanical damage if the deploy motor is allowed to run into the hard stop too much (although current limits prevent most damage).
     * @param enabled - Whether the soft limits should be enabled or not.
     */
    public default void setSoftLimits(boolean enabled) {}

    /**
     * Runs during {@link Intake#simulationPeriodic}.
     */
    public default void simIterate() {}
}
