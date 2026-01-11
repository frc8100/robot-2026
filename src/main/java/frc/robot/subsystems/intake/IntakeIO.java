package frc.robot.subsystems.intake;

import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {

        // Motor controller data for the intake motor
        public SparkMotorControllerData motorData = new SparkMotorControllerData();
        public boolean motorConnected = true;

        /**
         * Whether the intake is currently deployed.
         */
        public boolean isDeployed = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}
}
