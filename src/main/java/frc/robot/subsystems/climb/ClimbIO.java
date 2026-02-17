package frc.robot.subsystems.climb;

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
}
