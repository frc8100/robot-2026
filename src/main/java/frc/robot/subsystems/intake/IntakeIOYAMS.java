package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.CANIdConstants;
import frc.util.TunableValue;
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
    protected final SparkMax deployMotor = new SparkMax(CANIdConstants.DEPLOY_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark deployMotorWrapped = new WrappedSpark(deployMotor, IntakeConstants.deployMotorConfig);

    protected final Arm deployArm = new Arm(IntakeConstants.deployArmConfigFunction.apply(deployMotorWrapped));

    public final TunableValue.SparkPIDTunable tuning = TunableValue.SparkPIDTunable.fromWrapped(
        "Intake/Deploy",
        deployMotorWrapped
    );

    @Override
    public void setDeploySetpoint(Angle setpoint) {
        // deployArm.setMechanismPositionSetpoint(setpoint);

        // TODO: current disabled for "safety"
        deployMotorWrapped.setPosition(setpoint);
    }

    @Override
    public void runIntake(double speed) {
        intakeMotorWrapped.setDutyCycle(speed);
    }

    @Override
    public void runDeployVoltage(Voltage output) {
        deployMotorWrapped.setVoltage(output);
    }

    @Override
    public void runDeployDutyCycle(double output) {
        deployMotorWrapped.setDutyCycle(output);
    }

    @Override
    public void setDeployEncoderPosition(Angle position) {
        deployMotorWrapped.setEncoderPosition(position);
    }

    @Override
    public void removeSoftLimits() {
        SparkMaxConfig config = (SparkMaxConfig) deployMotorWrapped.getMotorControllerConfig();

        config.softLimit.forwardSoftLimitEnabled(false);
        config.softLimit.reverseSoftLimitEnabled(false);

        deployMotor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        System.out.println("Remove soft limits");
    }

    @Override
    public void applySoftLimits() {
        SparkMaxConfig config = (SparkMaxConfig) deployMotorWrapped.getMotorControllerConfig();

        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimitEnabled(true);

        deployMotor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        System.out.println("Apply soft limits");
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorConnected = intakeMotorWrapped.updateData(inputs.intakeMotorData);
        inputs.deployMotorConnected = deployMotorWrapped.updateData(inputs.deployMotorData);
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
