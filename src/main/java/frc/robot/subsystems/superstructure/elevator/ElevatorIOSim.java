package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

/**
 * The simulated IO for the elevator.
 */
public class ElevatorIOSim implements ElevatorIO {

    /**
     * Converts a position in radians to meters.
     * @param positionRadians - The position in radians.
     * @return The position in meters.
     */
    // private static double positionRadiansToMeters(double positionRadians) {
    //     return positionRadians * ElevatorConstants.ELEVATOR_MOTOR_POSITION_FACTOR;
    // }

    /**
     * The simulated elevator motor.
     */
    private final DCMotor motorGearbox = ElevatorConstants.SIM_MOTOR;

    /**
     * The profiled PID controller for the elevator.
     */
    private final ProfiledPIDController controller = new ProfiledPIDController(
        ElevatorConstants.SIM_KP,
        ElevatorConstants.SIM_KI,
        ElevatorConstants.SIM_KD,
        new TrapezoidProfile.Constraints(
            ElevatorConstants.ELEVATOR_SIM_MAX_VELOCITY.in(MetersPerSecond),
            ElevatorConstants.ELEVATOR_SIM_MAX_ACCELERATION.in(MetersPerSecondPerSecond)
        )
    );

    /**
     * The feedforward for the elevator.
     */
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
        ElevatorConstants.SIM_KS,
        ElevatorConstants.SIM_KG,
        ElevatorConstants.SIM_KV,
        ElevatorConstants.SIM_KA
    );

    /**
     * The simulated elevator.
     */
    private final ElevatorSim elevatorSim = new ElevatorSim(
        // The motor gearbox model
        motorGearbox,
        // The elevator's gear ratio
        ElevatorConstants.ELEVATOR_GEAR_RATIO,
        // The elevator's mass
        ElevatorConstants.ELEVATOR_MASS.in(Kilograms),
        // The elevator's drum radius
        ElevatorConstants.ELEVATOR_DRUM_RADIUS.in(Meters),
        // The elevator's minimum height
        // ElevatorConstants.Position.MIN_HEIGHT.in(Meters),
        0.0,
        // The elevator's maximum height
        // ElevatorConstants.Position.MAX_HEIGHT.in(Meters),
        2.0,
        // Whether to simulate gravity
        true,
        // The elevator's initial height
        ElevatorConstants.Position.MIN_HEIGHT.in(Meters),
        // Standard deviations for measurements
        0.0,
        0.0
    );

    /**
     * The motor simulation.
     */
    private final DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.createElevatorSystem(
            motorGearbox,
            ElevatorConstants.ELEVATOR_MASS.in(Kilograms),
            ElevatorConstants.ELEVATOR_DRUM_RADIUS.in(Meters),
            ElevatorConstants.ELEVATOR_GEAR_RATIO
        ),
        // (LinearSystem<N2, N1, N2>) elevatorSim,
        motorGearbox
    );

    /**
     * Whether the elevator is using a PID controller.
     */
    private boolean isUsingPID = true;

    public ElevatorIOSim() {
        // Reset the controller
        controller.reset(0.0);
    }

    @Override
    public void stop() {
        // Stop the elevator
        isUsingPID = false;

        elevatorSim.setInputVoltage(0.0);
        motorSim.setInputVoltage(0.0);
    }

    @Override
    public void setPosition(Distance position) {
        // Set the elevator position
        isUsingPID = true;
        controller.setGoal(position.in(Meters));
    }

    @Override
    public void setPosition(Angle angle) {
        // Set the elevator position
        isUsingPID = true;
        controller.setGoal(ElevatorConstants.getHeightFromMotorPosition(angle).in(Meters));
    }

    @Override
    public void runMotor(double motorInput) {
        // TODO: Run the motor
        // isUsingPID = false;
        // motorSim.setInputVoltage(MathUtil.clamp(motorInput, -12, 12));

        // Set the elevator position
        isUsingPID = true;
        controller.setGoal(
            controller.getSetpoint().position +
            motorInput * ElevatorConstants.AMOUNT_PER_FRAME * ElevatorConstants.ELEVATOR_MAX_OUTPUT
        );
    }

    @Override
    public void zeroEncoder(double value) {
        motorSim.setAngle(value);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Motor inputs
        inputs.connected = true;
        // inputs.positionRad = motorSim.getAngularPositionRad();
        inputs.positionRad = ElevatorConstants.getMotorPositionFromHeight(
            Meters.of(controller.getSetpoint().position)
        ).in(Radians);
        inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = motorSim.getInputVoltage();
        inputs.supplyCurrentAmps = motorSim.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = motorSim.getCurrentDrawAmps();
        // inputs.tempCelsius = 0.0;

        // Elevator inputs
        // TODO: fix elevator sim
        // inputs.height = elevatorSim.getPositionMeters();
        inputs.height = controller.getSetpoint().position;
        inputs.setpoint = controller.getSetpoint().position;
        // inputs.velocity = elevatorSim.getVelocityMetersPerSecond();
        inputs.velocity = controller.getSetpoint().velocity;
        inputs.setpointVelocity = controller.getSetpoint().velocity;

        inputs.isAtTarget = MathUtil.isNear(
            controller.getSetpoint().position,
            // elevatorSim.getPositionMeters(),
            controller.getGoal().position,
            ElevatorConstants.ELEVATOR_DISTANCE_TOLERANCE.in(Meters)
        );
    }

    @Override
    public void periodic() {
        double outputVoltage = 0.0;

        if (isUsingPID) {
            // Get the PID output
            double pidOutput = controller.calculate(elevatorSim.getPositionMeters());
            double feedforwardOutput =
                feedforward.calculate(controller.getSetpoint().velocity) * ElevatorConstants.SIM_KF;

            outputVoltage = pidOutput + feedforwardOutput;

            // Set the elevator input
            motorSim.setInputVoltage(MathUtil.clamp(outputVoltage, -12, 12));
        }
        // motorSim.update(0.02);

        // Update the elevator simulation
        // elevatorSim.setInputVoltage(outputVoltage);
        // elevatorSim.update(0.02);
    }
}
