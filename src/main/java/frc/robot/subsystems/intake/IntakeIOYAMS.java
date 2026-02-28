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

    // Intake motor
    protected final SparkMax rollerMotor = new SparkMax(CANIdConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark rollerMotorWrapped = new WrappedSpark(rollerMotor, IntakeConstants.intakeMotorConfig);

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
        deployMotorWrapped.setPosition(setpoint);
    }

    @Override
    public void runRollerDutyCycle(double speed) {
        rollerMotorWrapped.setDutyCycle(speed);
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
    public void setSoftLimits(boolean enabled) {
        SparkMaxConfig config = (SparkMaxConfig) deployMotorWrapped.getMotorControllerConfig();

        config.softLimit.forwardSoftLimitEnabled(enabled);
        config.softLimit.reverseSoftLimitEnabled(enabled);
        deployMotor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        System.out.println("[Deploy] " + (enabled ? "Applying" : "Removing") + " soft limits");
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerMotorConnected = rollerMotorWrapped.updateData(inputs.rollerMotorData);
        inputs.deployMotorConnected = deployMotorWrapped.updateData(inputs.deployMotorData);
    }
}
