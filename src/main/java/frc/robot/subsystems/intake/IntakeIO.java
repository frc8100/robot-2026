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

        // Deploy pneumatics
        // public boolean compressorEnabled = false;
        // public boolean isPressureSwitchValveNotFull = false;
        // public MutCurrent compressorCurrent = Amps.mutable(0.0);

        // public boolean deploySolenoidLeftState = false;
        // public boolean deploySolenoidRightState = false;

        // Deploy motor
        public SparkMotorControllerData deployMotorData = new SparkMotorControllerData();
        public boolean deployMotorConnected = true;

        // Motor controller data for the intake motor
        public SparkMotorControllerData intakeMotorData = new SparkMotorControllerData();
        public boolean intakeMotorConnected = true;
        /**
         * The measured deploy state of the intake. Independent of the desired deploy state; this is what the intake is actually doing. Should be determined by sensors on the intake.
         */
        // public MeasuredDeployState measuredDeployState = MeasuredDeployState.RETRACTED;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /**
     * Runs the intake at the given speed. Positive is intaking.
     * @param speed - The duty cycle speed from [-1, 1].
     */
    public default void runIntake(double speed) {}

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
     * Remove the soft limits (software on motor controller stops motor from moving past certain positions) on the deploy motor.
     * Used for testing. Also used for calibration sequence to return back to hard stop without worrying about soft limits.
     * Should be used with caution, as it can lead to mechanical damage if the deploy motor is allowed to run into the hard stop too much (although current limits prevent most damage)
     */
    public default void removeSoftLimits() {}

    /**
     * Reapplies the soft limits on the deploy motor after they have been removed.
     */
    public default void applySoftLimits() {}

    /**
     * Runs during {@link Intake#simulationPeriodic}.
     */
    public default void simIterate() {}
}
