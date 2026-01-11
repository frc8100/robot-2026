package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.util.SubsystemIOUtil;
import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import org.littletonrobotics.junction.AutoLog;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class TestArmSubsystemIO {

    @AutoLog
    public static class TestArmSubsystemIOInputs {

        public SparkMotorControllerData motorData = new SparkMotorControllerData();
        public boolean isMotorConnected = true;
    }

    private final SparkMax spark = new SparkMax(4, MotorType.kBrushless);
    private final SparkWrapper sparkSmartMotorController;

    public TestArmSubsystemIO(SmartMotorControllerConfig motorConfig) {
        sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), motorConfig);
    }

    public SmartMotorController getMotorController() {
        return sparkSmartMotorController;
    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(TestArmSubsystemIOInputs inputs) {
        inputs.isMotorConnected = SubsystemIOUtil.updateDataFromWrappedMotorController(
            inputs.motorData,
            sparkSmartMotorController
        );
    }
}
