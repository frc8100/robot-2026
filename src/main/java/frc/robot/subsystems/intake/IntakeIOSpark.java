package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.CANIdConstants;

public class IntakeIOSpark implements IntakeIO {

    // Deploy motor
    private final SparkMax deployMotor;
    private final RelativeEncoder deployEncoder;
    private final SparkClosedLoopController deployController;

    // Intake motor
    private final SparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;
    private final SparkClosedLoopController intakeController;

    public IntakeIOSpark() {
        // Initialize deploy motor
        deployMotor = new SparkMax(CANIdConstants.DEPLOY_MOTOR_ID, MotorType.kBrushless);
        deployEncoder = deployMotor.getEncoder();
        deployController = deployMotor.getClosedLoopController();

        // Initialize intake motor
        intakeMotor = new SparkMax(CANIdConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        intakeController = intakeMotor.getClosedLoopController();
    }
}
