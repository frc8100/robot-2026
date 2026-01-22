package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.CANIdConstants;
import frc.util.SubsystemIOUtil;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeIOYAMS implements IntakeIO {

    // Deploy motor
    protected final SparkMax deployMotor = new SparkMax(CANIdConstants.DEPLOY_MOTOR_ID, MotorType.kBrushless);
    protected final RelativeEncoder deployEncoder = deployMotor.getEncoder();
    protected final SparkClosedLoopController deployController = deployMotor.getClosedLoopController();

    protected final SparkWrapper deployMotorWrapped = new SparkWrapper(
        deployMotor,
        DCMotor.getNEO(1),
        IntakeConstants.deployMotorConfig
    );

    // Intake motor
    protected final SparkMax intakeMotor = new SparkMax(CANIdConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    protected final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    protected final SparkClosedLoopController intakeController = intakeMotor.getClosedLoopController();

    protected final SparkWrapper intakeMotorWrapped = new SparkWrapper(
        intakeMotor,
        DCMotor.getNEO(1),
        IntakeConstants.intakeMotorConfig
    );

    @Override
    public void runIntake(double speed) {
        intakeMotorWrapped.setDutyCycle(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.deployMotorConnected = SubsystemIOUtil.updateDataFromWrappedSparkOrSimulation(
            inputs.deployMotorData,
            deployMotorWrapped,
            deployMotor,
            deployEncoder,
            deployController
        );
        inputs.intakeMotorConnected = SubsystemIOUtil.updateDataFromWrappedSparkOrSimulation(
            inputs.intakeMotorData,
            intakeMotorWrapped,
            intakeMotor,
            intakeEncoder,
            intakeController
        );
    }
}
