package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.CANIdConstants;
import frc.util.WrappedSpark;
import yams.mechanisms.positional.Arm;

public class IntakeIOYAMS implements IntakeIO {

    // Deploy pneumatics
    // protected final Solenoid deploySolenoidLeft = new Solenoid(
    //     PneumaticsModuleType.CTREPCM,
    //     CANIdConstants.DEPLOY_SOLENOID_LEFT_CHANNEL
    // );
    // protected final Solenoid deploySolenoidRight = new Solenoid(
    //     PneumaticsModuleType.CTREPCM,
    //     CANIdConstants.DEPLOY_SOLENOID_RIGHT_CHANNEL
    // );
    // protected final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    // Intake motor
    protected final SparkMax intakeMotor = new SparkMax(CANIdConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark intakeMotorWrapped = new WrappedSpark(intakeMotor, IntakeConstants.intakeMotorConfig);

    // Deploy motor
    // protected final SparkMax deployMotor = new SparkMax(CANIdConstants.DEPLOY_MOTOR_ID, MotorType.kBrushless);
    // protected final WrappedSpark deployMotorWrapped = new WrappedSpark(deployMotor, IntakeConstants.deployMotorConfig);

    // protected final Arm deployArm = new Arm(IntakeConstants.deployArmConfigFunction.apply(deployMotorWrapped));

    @Override
    public void deploy() {
        // deploySolenoidLeft.set(true);
        // deploySolenoidRight.set(true);

        // deployArm.setMechanismPositionSetpoint(IntakeConstants.INTAKE_DEPLOYED_ANGLE);
    }

    @Override
    public void retract() {
        // deploySolenoidLeft.set(false);
        // deploySolenoidRight.set(false);

        // deployArm.setMechanismPositionSetpoint(IntakeConstants.INTAKE_RETRACTED_ANGLE);
    }

    @Override
    public void runIntake(double speed) {
        intakeMotorWrapped.setDutyCycle(speed);
    }

    @Override
    public void runDeployDutyCycle(double output) {
        // deployArm.setDutyCycleSetpoint(output);
    }

    @Override
    public void setDeployEncoderPosition(Angle position) {
        // deployMotorWrapped.setEncoderPosition(position);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorConnected = intakeMotorWrapped.updateData(inputs.intakeMotorData);
        // inputs.deployMotorConnected = deployMotorWrapped.updateData(inputs.deployMotorData);

        // inputs.deploySolenoidLeftState = deploySolenoidLeft.get();
        // inputs.deploySolenoidRightState = deploySolenoidRight.get();
        // inputs.measuredDeployState = (deploySolenoidLeft.get() && deploySolenoidRight.get())
        //     ? MeasuredDeployState.DEPLOYED
        //     : MeasuredDeployState.RETRACTED;

        // inputs.compressorEnabled = compressor.isEnabled();
        // inputs.isPressureSwitchValveNotFull = compressor.getPressureSwitchValue();
        // inputs.compressorCurrent.mut_replace(compressor.getCurrent(), Amps);
    }
}
