package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.MutCurrent;
import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    public enum MeasuredDeployState {
        DEPLOYED,
        RETRACTED,
        TRANSITION,
    }

    @AutoLog
    public static class IntakeIOInputs {

        // Deploy pneumatics
        public boolean compressorEnabled = false;
        public boolean isPressureSwitchValveNotFull = false;
        public MutCurrent compressorCurrent = Amps.mutable(0.0);

        // Motor controller data for the intake motor
        public SparkMotorControllerData intakeMotorData = new SparkMotorControllerData();
        public boolean intakeMotorConnected = true;

        /**
         * The measured deploy state of the intake. Independent of the desired deploy state; this is what the intake is actually doing. Should be determined by sensors on the intake.
         */
        public MeasuredDeployState measuredDeployState = MeasuredDeployState.RETRACTED;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}

    // test
    public default void runIntake(double speed) {}

    public default void deploy() {}

    public default void retract() {}

    public default void simIterate() {}
}
