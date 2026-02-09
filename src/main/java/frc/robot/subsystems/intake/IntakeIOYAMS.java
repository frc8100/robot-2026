package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.CANIdConstants;
import frc.util.WrappedSpark;

public class IntakeIOYAMS implements IntakeIO {

    // Deploy motor
    protected final SparkMax deployMotor = new SparkMax(CANIdConstants.DEPLOY_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark deployMotorWrapped = new WrappedSpark(deployMotor, IntakeConstants.deployMotorConfig);

    // Intake motor
    protected final SparkMax intakeMotor = new SparkMax(CANIdConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark intakeMotorWrapped = new WrappedSpark(intakeMotor, IntakeConstants.intakeMotorConfig);

    // protected final Solenoid deploySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    @Override
    public void deploy() {
        // deployMotorWrapped.setPosition(null);
    }

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
