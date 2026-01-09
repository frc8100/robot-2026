package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.util.SparkUtil.ifOk;
import static frc.util.SparkUtil.sparkStickyFault;
import static frc.util.SparkUtil.tryUntilOk;

// import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.claw.ClawConstants;
import frc.robot.subsystems.superstructure.claw.ClawIO.ClawIOInputs;
import frc.util.GenericSparkIO.GenericSparkIOConfig;
import frc.util.TunableValue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSpark implements ElevatorIO {

    // Leader turn motor variables
    private final SparkMax leaderMotor;
    private final RelativeEncoder leaderEncoder;
    private final SparkClosedLoopController leaderClosedLoopController;
    private SparkMaxConfig leaderConfig;

    // Follower turn motor variables
    private final SparkMax followerMotor;
    private SparkMaxConfig followerConfig;

    private final SparkLimitSwitch limitSwitch;

    /**
     * A debouncer for the connected state, to prevent flickering.
     */
    private final Debouncer leaderConnectedDebouncer = new Debouncer(0.5);

    /**
     * The setpoint of the elevator in radians.
     */
    private double radianSetpoint = 0.0;

    private boolean isUsingPID = true;

    private final ProfiledPIDController debugProfiledPIDController;

    /**
     * The config for the outtake motor.
     */
    private static class LeaderMotorConfig extends GenericSparkIOConfig {

        public LeaderMotorConfig() {
            super();
            // Override the default config
            this.idleMode = SparkBaseConfig.IdleMode.kBrake;
            this.inverted = ElevatorConstants.ELEVATOR_MOTOR_INVERTED;
            this.smartCurrentLimit = (int) ElevatorConstants.ELEVATOR_MOTOR_CURRENT_LIMIT.in(Amps);
            this.positionConversionFactor = ElevatorConstants.ELEVATOR_MOTOR_POSITION_FACTOR;
        }
    }

    // private static class FollowerMotorConfig extends GenericSparkIOConfig {

    //     public FollowerMotorConfig() {
    //         super();
    //         // Override the default config
    //         this.idleMode = SparkBaseConfig.IdleMode.kBrake;
    //         this.inverted = ElevatorConstants.ELEVATOR_MOTOR_INVERTED;
    //         this.smartCurrentLimit = (int) ElevatorConstants.ELEVATOR_MOTOR_CURRENT_LIMIT.in(Amps);
    //         this.positionConversionFactor = ElevatorConstants.ELEVATOR_MOTOR_POSITION_FACTOR;
    //     }
    // }

    public ElevatorIOSpark() {
        // Create the motor and configure it
        leaderMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        leaderEncoder = leaderMotor.getEncoder();
        leaderClosedLoopController = leaderMotor.getClosedLoopController();
        limitSwitch = leaderMotor.getReverseLimitSwitch();

        leaderConfig = new LeaderMotorConfig().getConfig();

        leaderConfig.closedLoop
            .pid(
                ElevatorConstants.ELEVATOR_KP.get(),
                ElevatorConstants.ELEVATOR_KI.get(),
                ElevatorConstants.ELEVATOR_KD.get()
            )
            .outputRange(-ElevatorConstants.ELEVATOR_MAX_OUTPUT, ElevatorConstants.ELEVATOR_MAX_OUTPUT);

        leaderConfig.closedLoop.maxMotion
            .maxVelocity(ElevatorConstants.ELEVATOR_MAX_ANGULAR_VELOCITY.in(RadiansPerSecond))
            .maxAcceleration(ElevatorConstants.ELEVATOR_MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));

        leaderConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen).reverseLimitSwitchEnabled(false);

        // Apply the config
        tryUntilOk(leaderMotor, 5, () ->
            leaderMotor.configure(
                leaderConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
        );

        // Reset the encoder
        tryUntilOk(leaderMotor, 5, () -> leaderEncoder.setPosition(0.0));

        // Create the follower motor and configure it
        followerMotor = new SparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless);

        // followerConfig = new FollowerMotorConfig().getConfig();
        followerConfig = leaderConfig;

        followerConfig.follow(leaderMotor);

        // Apply the config
        tryUntilOk(followerMotor, 5, () ->
            followerMotor.configure(
                followerConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
        );

        debugProfiledPIDController = new ProfiledPIDController(
            ElevatorConstants.ELEVATOR_KP.get(),
            ElevatorConstants.ELEVATOR_KI.get(),
            ElevatorConstants.ELEVATOR_KD.get(),
            ElevatorConstants.PROFILED_ELEVATOR_CONSTRAINTS
        );

        TunableValue.addRefreshConfigConsumer(this::refreshConfig);
    }

    public void refreshConfig() {
        // Refresh the config for the motor
        leaderConfig.closedLoop.pid(
            ElevatorConstants.ELEVATOR_KP.get(),
            ElevatorConstants.ELEVATOR_KI.get(),
            ElevatorConstants.ELEVATOR_KD.get()
        );
        // .outputRange(-ElevatorConstants.ELEVATOR_MAX_OUTPUT, ElevatorConstants.ELEVATOR_MAX_OUTPUT);

        debugProfiledPIDController.setPID(
            ElevatorConstants.ELEVATOR_KP.get(),
            ElevatorConstants.ELEVATOR_KI.get(),
            ElevatorConstants.ELEVATOR_KD.get()
        );

        tryUntilOk(leaderMotor, 5, () ->
            leaderMotor.configure(
                leaderConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
        );
    }

    @Override
    public void runMotor(double motorInput) {
        // Use PID
        double positionCurrent = leaderEncoder.getPosition();

        double percentOutput = motorInput * ElevatorConstants.AMOUNT_PER_FRAME;

        // TODO:
        // If it is at the top, slow
        if (positionCurrent > ElevatorConstants.ELEVATOR_MAX_POSITION.in(Radians)) {
            percentOutput *= 0.075;
        } else if (positionCurrent > ElevatorConstants.ELEVATOR_TOP_THRESHOLD.in(Radians)) {
            percentOutput *= ElevatorConstants.ELEVATOR_TOP_INPUT;
        } else {
            percentOutput *= ElevatorConstants.ELEVATOR_MAX_OUTPUT;
        }

        // TODO: constant
        radianSetpoint += percentOutput;
        // isUsingPID = false;
        // double positionCurrent = encoder.getPosition();

        // motor.set(percentOutput);

        // Log
        // Logger.recordOutput("Elevator/motorInput", motorInput);
        // Logger.recordOutput("Elevator/percentOutput", percentOutput);
    }

    @Override
    public void stop() {
        leaderMotor.stopMotor();
    }

    @Override
    public void resetSetpointToCurrentPosition() {
        radianSetpoint = leaderEncoder.getPosition();
    }

    @Override
    public void zeroEncoder(double value) {
        leaderEncoder.setPosition(value);
        radianSetpoint = value;
    }

    @Override
    public void setPosition(Distance position) {
        // Set the position of the turn motor
        radianSetpoint = ElevatorConstants.getMotorPositionFromHeight(position).in(Radians);

        isUsingPID = true;
    }

    @Override
    public void setPosition(SuperstructureConstants.Level level) {
        radianSetpoint = level.getElevatorRadian();

        isUsingPID = true;
    }

    @Override
    public void setPosition(Angle position) {
        radianSetpoint = position.in(Radians);

        isUsingPID = true;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Clamp
        radianSetpoint = MathUtil.clamp(
            radianSetpoint,
            ElevatorConstants.ELEVATOR_MIN_POSITION.in(Radians),
            ElevatorConstants.ELEVATOR_MAX_POSITION.in(Radians)
        );

        inputs.setpoint = radianSetpoint;
        inputs.isAtTarget = MathUtil.isNear(
            leaderEncoder.getPosition(),
            radianSetpoint,
            ElevatorConstants.ELEVATOR_RAD_TOLERANCE.in(Radians)
        );
        inputs.isAtTargetNotNearer = MathUtil.isNear(
            leaderEncoder.getPosition(),
            radianSetpoint,
            ElevatorConstants.ELEVATOR_RAD_TOLERANCE_NOT_NEARER.in(Radians)
        );

        debugProfiledPIDController.setGoal(radianSetpoint);
        inputs.trapezoidalVoltage = debugProfiledPIDController.calculate(leaderEncoder.getPosition());

        if (isUsingPID) {
            leaderClosedLoopController.setReference(radianSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
            // leaderClosedLoopController.setReference(debugProfiledPIDController.getSetpoint().position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        // Reset spark sticky fault
        sparkStickyFault = false;

        // Update inputs for the turn motor

        // Set the position and velocity
        ifOk(leaderMotor, leaderEncoder::getPosition, position -> inputs.positionRad = position);
        ifOk(leaderMotor, leaderEncoder::getVelocity, velocity -> inputs.velocityRadPerSec = velocity);
        ifOk(leaderMotor, leaderEncoder::getPosition, position ->
            inputs.height = position * ElevatorConstants.ELEVATOR_RADIANS_TO_METERS
        );

        // Set the supply current based on the bus voltage multiplied by the applied output
        ifOk(leaderMotor, new DoubleSupplier[] { leaderMotor::getBusVoltage, leaderMotor::getAppliedOutput }, x ->
            inputs.appliedVolts = x[0] * x[1]
        );

        // Set the torque current
        ifOk(leaderMotor, leaderMotor::getOutputCurrent, current -> inputs.torqueCurrentAmps = current);

        // Set the connected state with a debouncer
        inputs.connected = leaderConnectedDebouncer.calculate(!sparkStickyFault);

        // Set the temperature
        // ifOk(leaderMotor, leaderMotor::getMotorTemperature, temp -> inputs.tempCelsius = temp);

        inputs.isAtBottom = limitSwitch.isPressed();

        inputs.trapezoidalSetpoint = debugProfiledPIDController.getSetpoint().position;
        inputs.trapezoidalSetpointVelocity = debugProfiledPIDController.getSetpoint().velocity;
    }
}
