package frc.robot.subsystems.superstructure.claw;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.util.TunableValue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * IO implementation for spark motors
 */
public class ClawIOSpark implements ClawIO {

    // Turn motor variables
    /**
     * The motor for the rotation of the claw.
     */
    private final SparkMax angleMotor;

    /**
     * The relative encoder for the rotation motor.
     */
    private final RelativeEncoder angleEncoder;

    /**
     * The closed loop controller for the rotation motor.
     */
    private final SparkClosedLoopController angleClosedLoopController;

    /**
     * A debouncer for the connected state, to prevent flickering.
     */
    private final Debouncer angleConnectedDebouncer = new Debouncer(0.5);

    // Outtake motor variables
    /**
     * The motor for the outtake of the claw.
     */
    private final SparkMax outtakeMotor;

    private final SparkLimitSwitch outtakeLimitSwitch;

    /**
     * The relative encoder for the outtake motor.
     */
    private final RelativeEncoder outtakeEncoder;

    /**
     * The closed loop controller for the outtake motor.
     */
    // private final SparkClosedLoopController outtakeClosedLoopController;
    private final ProfiledPIDController angleProfiledClosedLoopController;

    private double radianSetpoint = 0.0;

    /**
     * A debouncer for the connected state, to prevent flickering.
     */
    private final Debouncer outtakeConnectedDebouncer = new Debouncer(0.5);

    /**
     * The configuration for the angle motor.
     */
    // TODO: Combine into generic spark config
    private SparkMaxConfig angleConfig;

    /**
     * The config for the turn motor.
     */
    private static class AngleConfig extends GenericSparkIOConfig {

        public AngleConfig() {
            super();
            // Override the default config
            this.idleMode = SparkBaseConfig.IdleMode.kBrake;
            this.inverted = ClawConstants.IS_ANGLE_MOTOR_INVERTED;
            this.smartCurrentLimit = (int) ClawConstants.ANGLE_MOTOR_CURRENT_LIMIT.in(Amps);
            this.positionConversionFactor = ClawConstants.ANGLE_ENCODER_POSITION_FACTOR;
        }
    }

    /**
     * The config for the outtake motor.
     */
    private static class OuttakeConfig extends GenericSparkIOConfig {

        public OuttakeConfig() {
            super();
            // Override the default config
            this.idleMode = SparkBaseConfig.IdleMode.kBrake;
            this.inverted = ClawConstants.IS_OUTTAKE_MOTOR_INVERTED;
            this.smartCurrentLimit = (int) ClawConstants.OUTTAKE_MOTOR_CURRENT_LIMIT.in(Amps);
            this.positionConversionFactor = ClawConstants.OUTTAKE_ENCODER_POSITION_FACTOR;
        }
    }

    public ClawIOSpark() {
        // Create the motor and configure it
        angleMotor = new SparkMax(ClawConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        angleClosedLoopController = angleMotor.getClosedLoopController();

        angleConfig = new AngleConfig().getConfig();

        // Apply PID config for the angle motor
        angleConfig.closedLoop
            .pid(ClawConstants.ANGLE_KP.get(), ClawConstants.ANGLE_KI.get(), ClawConstants.ANGLE_KD.get())
            .outputRange(-ClawConstants.MAX_ANGLE_POWER, ClawConstants.MAX_ANGLE_POWER);

        angleProfiledClosedLoopController = new ProfiledPIDController(
            ClawConstants.PROFILED_ANGLE_KP.get(),
            ClawConstants.PROFILED_ANGLE_KI.get(),
            ClawConstants.PROFILED_ANGLE_KD.get(),
            ClawConstants.PROFILED_ANGLE_CONSTRAINTS
        );

        // Apply the config
        tryUntilOk(angleMotor, 5, () ->
            angleMotor.configure(
                angleConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
        );

        // Reset the encoder
        tryUntilOk(angleMotor, 5, () -> angleEncoder.setPosition(0.0));

        // Create the outtake motor and configure it
        outtakeMotor = new SparkMax(ClawConstants.OUTTAKE_MOTOR_ID, MotorType.kBrushless);
        outtakeEncoder = outtakeMotor.getEncoder();
        outtakeLimitSwitch = outtakeMotor.getReverseLimitSwitch();
        // outtakeClosedLoopController = outtakeMotor.getClosedLoopController();

        SparkMaxConfig outtakeConfig = new OuttakeConfig().getConfig();

        outtakeConfig.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchEnabled(false);

        // Apply the config
        tryUntilOk(outtakeMotor, 5, () ->
            outtakeMotor.configure(
                outtakeConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
        );

        // Reset the encoder
        tryUntilOk(outtakeMotor, 5, () -> outtakeEncoder.setPosition(0.0));

        TunableValue.addRefreshConfigConsumer(this::refreshConfig);
    }

    public void refreshConfig() {
        // Refresh the config for the motor
        angleConfig.closedLoop.pid(
            ClawConstants.ANGLE_KP.get(),
            ClawConstants.ANGLE_KI.get(),
            ClawConstants.ANGLE_KD.get()
        );

        angleProfiledClosedLoopController.setPID(
            ClawConstants.PROFILED_ANGLE_KP.get(),
            ClawConstants.PROFILED_ANGLE_KI.get(),
            ClawConstants.PROFILED_ANGLE_KD.get()
        );

        // TODO: add max velocity and acceleration to tunable config

        tryUntilOk(angleMotor, 5, () ->
            angleMotor.configure(
                angleConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
        );
    }

    @Override
    public void runOuttake(double motorInput) {
        // Apply deadband
        motorInput = MathUtil.applyDeadband(motorInput, ClawConstants.CONTROLLER_DEADBAND);

        // Run the motor
        double percentOutput = ClawConstants.OUTTAKE_MAX_OUTPUT * motorInput;
        outtakeMotor.set(percentOutput);
        // Log
        // Logger.recordOutput("Claw/motorInput", motorInput);
        // Logger.recordOutput("Claw/percentOutput", percentOutput);
    }

    @Override
    public void stop() {
        angleMotor.stopMotor();
        outtakeMotor.stopMotor();
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        radianSetpoint = rotation.getRadians();
    }

    @Override
    public void increaseTurnPosition(double valueToIncreaseBy) {
        radianSetpoint += valueToIncreaseBy;
    }

    @Override
    public void resetSetpointToCurrentPosition() {
        radianSetpoint = angleEncoder.getPosition();
    }

    @Override
    public void zeroEncoder(double value) {
        angleEncoder.setPosition(value);
        radianSetpoint = 0.0;
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        // TODO: Log empty setpoints when disabled

        radianSetpoint = MathUtil.clamp(
            radianSetpoint,
            ClawConstants.MIN_CLAW_ANGLE.in(Radians),
            ClawConstants.MAX_CLAW_ANGLE.in(Radians)
        );

        inputs.turnSetpointRad = radianSetpoint;

        // Set the position of the turn motor
        // angleClosedLoopController.setReference(radianSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        angleProfiledClosedLoopController.setGoal(radianSetpoint);
        angleProfiledClosedLoopController.calculate(angleEncoder.getPosition());
        angleClosedLoopController.setReference(
            angleProfiledClosedLoopController.getSetpoint().position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );

        // Reset spark sticky fault
        sparkStickyFault = false;

        // Update inputs for the turn motor

        // Set the position and velocity
        ifOk(angleMotor, angleEncoder::getPosition, position -> inputs.turnPositionRad = position);
        ifOk(angleMotor, angleEncoder::getVelocity, velocity -> inputs.turnVelocityRadPerSec = velocity);
        inputs.turnTrapezoidalTargetVelocityRadPerSec = angleProfiledClosedLoopController.getSetpoint().velocity;
        inputs.turnTrapezoidalTargetPositionRad = angleProfiledClosedLoopController.getSetpoint().position;

        // Set the supply current based on the bus voltage multiplied by the applied output
        ifOk(angleMotor, new DoubleSupplier[] { angleMotor::getBusVoltage, angleMotor::getAppliedOutput }, x ->
            inputs.turnAppliedVolts = x[0] * x[1]
        );

        // Set the torque current
        ifOk(angleMotor, angleMotor::getOutputCurrent, current -> inputs.turnTorqueCurrentAmps = current);

        // Set the connected state with a debouncer
        inputs.turnConnected = angleConnectedDebouncer.calculate(!sparkStickyFault);

        // Set the temperature
        // ifOk(angleMotor, angleMotor::getMotorTemperature, temp -> inputs.turnTempCelsius = temp);

        // Update inputs for the outtake motor

        // Set the position and velocity
        ifOk(outtakeMotor, outtakeEncoder::getPosition, position -> inputs.outtakePositionRad = position);
        ifOk(outtakeMotor, outtakeEncoder::getVelocity, velocity -> inputs.outtakeVelocityRadPerSec = velocity);

        // Set the supply current based on the bus voltage multiplied by the applied output
        ifOk(outtakeMotor, new DoubleSupplier[] { outtakeMotor::getBusVoltage, outtakeMotor::getAppliedOutput }, x ->
            inputs.outtakeAppliedVolts = x[0] * x[1]
        );

        // Set the torque current
        ifOk(outtakeMotor, outtakeMotor::getOutputCurrent, current -> inputs.outtakeTorqueCurrentAmps = current);

        // Set the connected state with a debouncer
        inputs.outtakeConnected = outtakeConnectedDebouncer.calculate(!sparkStickyFault);

        inputs.isCoralInClaw = outtakeLimitSwitch.isPressed();
    }
}
