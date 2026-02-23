package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {

        public SparkMotorControllerData leftClimbMotorData = new SparkMotorControllerData();
        public boolean leftClimbMotorConnected = true;

        public SparkMotorControllerData rightClimbMotorData = new SparkMotorControllerData();
        public boolean rightClimbMotorConnected = true;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimbIOInputs inputs) {}

    public default void runClimbDutyCycle(double output) {}

    public default void setClimbEncoderPosition(Rotation2d position) {
        setClimbEncoderPosition(position.getMeasure());
    }

    public default void setClimbEncoderPosition(Angle position) {}

    public default void setClimbTarget(Angle targetPosition) {}

    public default void simIterate() {}
}
