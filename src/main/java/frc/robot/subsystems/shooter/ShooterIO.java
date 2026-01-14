package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {

        // Motor controller data for the intake motor
        public SparkMotorControllerData motorData = new SparkMotorControllerData();
        public boolean motorConnected = true;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setMotorSpeed(AngularVelocity speed) {}

    public default void setMotorSpeed(double speedRadiansPerSecond) {}
}
