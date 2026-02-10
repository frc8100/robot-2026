package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.CANIdConstants;
import frc.util.WrappedSpark;

public class IntakeIOYAMS implements IntakeIO {

    // Deploy pneumatics
    protected final Solenoid deploySolenoidLeft = new Solenoid(
        PneumaticsModuleType.CTREPCM,
        CANIdConstants.DEPLOY_SOLENOID_LEFT_CHANNEL
    );
    protected final Solenoid deploySolenoidRight = new Solenoid(
        PneumaticsModuleType.CTREPCM,
        CANIdConstants.DEPLOY_SOLENOID_RIGHT_CHANNEL
    );
    protected final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    // Intake motor
    protected final SparkMax intakeMotor = new SparkMax(CANIdConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark intakeMotorWrapped = new WrappedSpark(intakeMotor, IntakeConstants.intakeMotorConfig);

    // protected final Solenoid deploySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    @Override
    public void deploy() {
        // deployMotorWrapped.setDutyCycle(0.5);
        deploySolenoidLeft.set(true);
        deploySolenoidRight.set(true);
    }

    @Override
    public void runIntake(double speed) {
        intakeMotorWrapped.setDutyCycle(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorConnected = intakeMotorWrapped.updateData(inputs.intakeMotorData);

        inputs.measuredDeployState = (deploySolenoidLeft.get() && deploySolenoidRight.get())
            ? MeasuredDeployState.DEPLOYED
            : MeasuredDeployState.RETRACTED;

        inputs.compressorEnabled = compressor.isEnabled();
        inputs.isPressureSwitchValveNotFull = compressor.getPressureSwitchValue();
        inputs.compressorCurrent.mut_replace(compressor.getCurrent(), Amps);
    }
}
