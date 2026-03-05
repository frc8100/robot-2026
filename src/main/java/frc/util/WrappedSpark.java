package frc.util;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.util.SubsystemIOUtil.SparkMotorControllerData;
import java.util.Optional;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.simulation.DCMotorSimSupplier;

/**
 * A wrapper around {@link SparkWrapper} that adds additional functionality for our specific use cases.
 */
// TODO: rename
public class WrappedSpark extends SparkWrapper {

    public static class CustomLoggedPIDController extends PIDController {

        // TODO: make thread safe
        // public final MutVoltage lastOutput = Volts.mutable(0.0);
        // public volatile MutVoltage lastOutput = Volts.mutable(0.0);
        public volatile double lastOutput = 0.0;
        public volatile double lastSetpoint = 0.0;

        public CustomLoggedPIDController(double kP, double kI, double kD) {
            super(kP, kI, kD);
        }

        @Override
        public double calculate(double measurement, double setpoint) {
            double output = super.calculate(measurement, setpoint);

            // lastOutput.mut_replace(output, Volts);
            lastOutput = output;
            lastSetpoint = setpoint;

            return output;
        }
    }

    protected final SparkMax motor;
    protected final RelativeEncoder encoder;
    protected final SparkClosedLoopController controller;

    // TODO: Workaround for using trapezoid profile
    // protected final Optional<ProfiledPIDController> customMotionProfileController = Optional.empty();

    public WrappedSpark(SparkMax motor, DCMotor model, SmartMotorControllerConfig config) {
        // motor = new SparkMax(canId, MotorType.kBrushless);
        // encoder = motor.getEncoder();
        // controller = motor.getClosedLoopController();

        // motorWrapped = new SparkWrapper(motor, DCMotor.getNEO(1), config);

        super(motor, model, config);
        this.motor = motor;
        this.encoder = motor.getEncoder();
        this.controller = motor.getClosedLoopController();
    }

    public WrappedSpark(SparkMax motor, SmartMotorControllerConfig config) {
        this(motor, DCMotor.getNEO(1), config);
    }

    /**
     * Updates the given data object from this wrapped motor controller.
     * @param dataToUpdate - The data object to update. This is modified in place.
     * @return Whether the data was successfully updated (no sticky faults).
     */
    public boolean updateData(SparkMotorControllerData dataToUpdate) {
        return SubsystemIOUtil.updateDataFromWrappedMotorController(dataToUpdate, this);
    }

    /**
     * Sets up custom simulation for this motor controller. This should be called in the simulation IO class constructor.
     */
    public void setupCustomSimulation() {
        SimulatedBattery.addElectricalAppliances(this::getCustomSupplyCurrent);
    }

    // Hacky methods

    public double getLastPIDOutput() {
        if (super.m_pid.isEmpty()) {
            DriverStation.reportWarning(
                "PID controller is not present for motor " + motor.getDeviceId() + ", cannot get last PID output",
                false
            );
            return 0.0;
        }
        if (!(super.m_pid.get() instanceof CustomLoggedPIDController)) {
            DriverStation.reportWarning(
                "PID controller is not a CustomLoggedPIDController for motor " +
                motor.getDeviceId() +
                ", cannot get last PID output",
                false
            );
            return 0.0;
        }

        return ((CustomLoggedPIDController) super.m_pid.get()).lastOutput;
    }

    public double getLastPIDSetpoint() {
        if (super.m_pid.isEmpty()) {
            DriverStation.reportWarning(
                "PID controller is not present for motor " + motor.getDeviceId() + ", cannot get last PID setpoint",
                false
            );
            return 0.0;
        }
        if (!(super.m_pid.get() instanceof CustomLoggedPIDController)) {
            DriverStation.reportWarning(
                "PID controller is not a CustomLoggedPIDController for motor " +
                motor.getDeviceId() +
                ", cannot get last PID setpoint",
                false
            );
            return 0.0;
        }

        return ((CustomLoggedPIDController) super.m_pid.get()).lastSetpoint;
    }

    /**
     * Normally YAMS only uses a closed loop controller thread (for sparks) when in LQR or exponential mode. This force sets up the closed loop controller thread for this motor.
     */
    public void forceSetupClosedLoopController() {
        if (m_closedLoopControllerThread != null) {
            DriverStation.reportWarning(
                "Closed loop controller thread already running for motor " + motor.getDeviceId(),
                false
            );
            return;
        }

        m_closedLoopControllerThread = new Notifier(super::iterateClosedLoopController);
    }

    public TrapezoidProfile.State currentState = new TrapezoidProfile.State(0.0, 0.0);
    public final MutVoltage lastFeedforward = Volts.mutable(0.0);

    public void setSetpointPositionWithoutOutput(Angle setpointPosition) {
        super.setpointPosition = Optional.ofNullable(setpointPosition);
        super.setpointVelocity = Optional.empty();
    }

    public void overrideCurrentState(Angle position, AngularVelocity velocity) {
        currentState = new TrapezoidProfile.State(position.in(Rotations), velocity.in(RotationsPerSecond));
    }

    public void overrideCurrentState(AngularVelocity velocity, AngularAcceleration acceleration) {
        currentState = new TrapezoidProfile.State(
            velocity.in(RotationsPerSecond),
            acceleration.in(RotationsPerSecondPerSecond)
        );
    }

    public void iterateCustomMotionProfile() {
        if (super.m_trapezoidProfile.isEmpty()) {
            return;
        }

        TrapezoidProfile.State setpointState = currentState;
        ControlType controlType = ControlType.kPosition;

        if (super.setpointPosition.isPresent()) {
            // Position control
            setpointState = new TrapezoidProfile.State(
                super.setpointPosition.orElse(Rotations.zero()).in(Rotations),
                0.0
            );
            controlType = ControlType.kPosition;
        } else if (super.setpointVelocity.isPresent()) {
            // Velocity control
            setpointState = new TrapezoidProfile.State(
                super.setpointVelocity.orElse(RotationsPerSecond.zero()).in(RotationsPerSecond),
                0.0
            );
            controlType = ControlType.kVelocity;
        } else {
            // No setpoint, do nothing
            return;
        }

        TrapezoidProfile profile = super.m_trapezoidProfile.get();

        currentState = profile.calculate(0.02, currentState, setpointState);

        TrapezoidProfile.State nextState = profile.calculate(0.02, currentState, setpointState);

        super.m_config
            .getSimpleFeedforward()
            .ifPresent(ff -> {
                ff.setKs(0.0);
                lastFeedforward.mut_replace(
                    ff.calculateWithVelocities(currentState.velocity, nextState.velocity),
                    Volts
                );
            });
        super.m_config
            .getArmFeedforward()
            .ifPresent(ff -> {
                ff.setKs(0.0);
                ff.setKg(0.0);
                lastFeedforward.mut_replace(
                    ff.calculateWithVelocities(currentState.position, currentState.velocity, nextState.velocity),
                    Volts
                );
            });
        super.m_config
            .getElevatorFeedforward()
            .ifPresent(ff -> {
                ff.setKs(0.0);
                ff.setKg(0.0);
                lastFeedforward.mut_replace(
                    ff.calculateWithVelocities(currentState.velocity, nextState.velocity),
                    Volts
                );
            });

        controller.setSetpoint(
            currentState.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            lastFeedforward.in(Volts),
            ArbFFUnits.kVoltage
        );
    }

    // TODO: this is a bit of a hack, but it allows us to use the existing SparkWrapper closed loop control methods while also allowing us to set the setpoint velocity in simulation (since YAMS does not simulate closed loop control of velocity)
    public void setCustomSetpointVelocity(AngularVelocity velocity) {
        super.setpointVelocity = Optional.ofNullable(velocity);
    }

    /**
     * Gets the supply current of the motor in amps.
     * Adapted from {@link org.ironmaple.simulation.motorsims.MapleMotorSim#getSupplyCurrent}
     * @return The supply current of the motor in amps.
     */
    public Current getCustomSupplyCurrent() {
        return super.getStatorCurrent().times(super.getVoltage().in(Volts) / (RobotController.getBatteryVoltage()));
    }
}
