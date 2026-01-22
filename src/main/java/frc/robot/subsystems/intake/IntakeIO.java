package frc.robot.subsystems.intake;

import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {

        // Motor controller data for the intake motor
        public SparkMotorControllerData deployMotorData = new SparkMotorControllerData();
        public boolean deployMotorConnected = true;

        public SparkMotorControllerData intakeMotorData = new SparkMotorControllerData();
        public boolean intakeMotorConnected = true;

        /**
         * Whether the intake is currently deployed.
         */
        public boolean isDeployed = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}

    // test
    public default void runIntake(double speed) {}
}
