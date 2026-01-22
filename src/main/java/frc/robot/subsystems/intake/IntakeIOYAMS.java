package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.CANIdConstants;
import frc.util.WrappedSpark;

public class IntakeIOYAMS implements IntakeIO {

    // Deploy motor
    protected final SparkMax deployMotor = new SparkMax(CANIdConstants.DEPLOY_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark deployMotorWrapped = new WrappedSpark(deployMotor, IntakeConstants.deployMotorConfig);

    // Intake motor
    protected final SparkMax intakeMotor = new SparkMax(CANIdConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark intakeMotorWrapped = new WrappedSpark(intakeMotor, IntakeConstants.intakeMotorConfig);

    @Override
    public void runIntake(double speed) {
        intakeMotorWrapped.setDutyCycle(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.deployMotorConnected = deployMotorWrapped.updateData(inputs.deployMotorData);
        inputs.intakeMotorConnected = intakeMotorWrapped.updateData(inputs.intakeMotorData);
    }
}
