package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.CANIdConstants;
import frc.util.TunableValue;
import frc.util.WrappedSpark;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.positional.Arm;

public class IntakeIOYAMS implements IntakeIO {

    // Intake motor
    protected final SparkMax rollerMotor = new SparkMax(CANIdConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark rollerMotorWrapped = new WrappedSpark(rollerMotor, IntakeConstants.intakeMotorConfig);

    // Deploy motor
    protected final SparkMax deployMotor = new SparkMax(CANIdConstants.DEPLOY_MOTOR_ID, MotorType.kBrushless);
    protected final WrappedSpark deployMotorWrapped = new WrappedSpark(deployMotor, IntakeConstants.deployMotorConfig);

    protected final Arm deployArm = new Arm(IntakeConstants.deployArmConfigFunction.apply(deployMotorWrapped));

    protected boolean isUsingClosedLoopControl = true;

    public final TunableValue.SparkPIDTunable deployTunablePID = TunableValue.SparkPIDTunable.fromWrapped(
        "Intake/Deploy",
        deployMotorWrapped
    );
    public final TunableValue.SparkFeedforwardTunable deployTunableFF = new TunableValue.SparkFeedforwardTunable(
        "Intake/Deploy",
        deployMotorWrapped
    );

    public IntakeIOYAMS() {
        // deployMotorWrapped.setEncoderPosition(IntakeConstants.INTAKE_RETRACTED_ANGLE);
        // deployMotorWrapped.forceSetupClosedLoopController();
        // deployMotorWrapped.startClosedLoopController();

        enableClosedLoopControl();
    }

    private void disableClosedLoopControl() {
        isUsingClosedLoopControl = false;
        // deployMotorWrapped.stopClosedLoopController();
    }

    private void enableClosedLoopControl() {
        if (isUsingClosedLoopControl) {
            return;
        }

        isUsingClosedLoopControl = true;
        deployMotorWrapped.overrideCurrentState(
            deployMotorWrapped.getMechanismPosition(),
            deployMotorWrapped.getMechanismVelocity()
        );
        // deployMotorWrapped.startClosedLoopController();
    }

    @Override
    public void setDeploySetpoint(Angle setpoint) {
        enableClosedLoopControl();
        deployMotorWrapped.setSetpointPositionWithoutOutput(setpoint);
    }

    @Override
    public void runRollerDutyCycle(double speed) {
        rollerMotorWrapped.setDutyCycle(speed);
    }

    @Override
    public void runDeployVoltage(Voltage output) {
        disableClosedLoopControl();
        deployMotorWrapped.setVoltage(output);
    }

    @Override
    public void runDeployDutyCycle(double output) {
        disableClosedLoopControl();
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

        inputs.deployMotorData.setpointAngle.mut_replace(deployMotorWrapped.currentState.position, Rotations);

        // test
        Logger.recordOutput("Intake/DeploySetpointState", deployMotorWrapped.currentState);

        if (isUsingClosedLoopControl) {
            deployMotorWrapped.iterateCustomMotionProfile();
        }
    }
}
