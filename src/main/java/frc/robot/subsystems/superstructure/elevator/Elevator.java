package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.claw.ClawConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The elevator subsystem. This subsystem is responsible for controlling the elevator.
 */
public class Elevator extends SubsystemBase {

    /**
     * The IO interface for the elevator.
     */
    // todo: protect
    public final ElevatorIO io;

    /**
     * The inputs for the elevator.
     */
    protected final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Trigger whenElevatorIsAtBottom;

    /**
     * Creates a new Elevator subsystem.
     * @param io - The IO interface for the elevator.
     */
    public Elevator(ElevatorIO io) {
        this.io = io;

        whenElevatorIsAtBottom = new Trigger(this::isElevatorAtBottom);
    }

    /**
     * @return Whether the elevator is at the target position, within a tolerance.
     */
    @AutoLogOutput(key = "Elevator/IsElevatorAtTarget")
    public boolean isElevatorAtTarget() {
        return inputs.isAtTarget;
    }

    /**
     * @return Whether the elevator is at the bottom
     */
    @AutoLogOutput(key = "Elevator/IsElevatorAtBottom")
    public boolean isElevatorAtBottom() {
        return inputs.isAtBottom;
    }

    // TODO: refactor
    public Command getUpOrDown(DoubleSupplier valueSupplier) {
        return new RunCommand(
            () -> runMotor(MathUtil.applyDeadband(valueSupplier.getAsDouble(), ClawConstants.CONTROLLER_DEADBAND)),
            this
        );
    }

    /**
     * @return The position of the elevator as an angle of the motor.
     * ! IMPORTANT: This is not the linear position of the elevator.
     */
    public Angle getAngularPosition() {
        return Radians.of(inputs.positionRad);
    }

    @Override
    public void periodic() {
        io.periodic();

        // Update inputs
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    /**
     * Stops the elevator.
     */
    public void stop() {
        io.stop();
    }

    /**
     * Sets the desired elevator position.
     * @param position - The desired elevator position.
     */
    public void setPosition(Distance position) {
        io.setPosition(position);
    }

    public void setPosition(SuperstructureConstants.Level level) {
        io.setPosition(level);
    }

    public void setPosition(Angle angle) {
        io.setPosition(angle);
    }

    /**
     * Runs the elevator motor at the given input.
     * @param motorInput - The input to the motor from -1 to 1.
     */
    public void runMotor(double motorInput) {
        io.runMotor(motorInput);
    }

    /**
     * @return A command to set the elevator to a certain position.
     */
    public Command getPositionCommand(Distance position) {
        return new InstantCommand(() -> setPosition(position), this);
    }

    public Command getPositionCommand(SuperstructureConstants.Level level) {
        return new InstantCommand(() -> setPosition(level), this);
    }

    public Command getPositionCommand(Angle angle) {
        return new InstantCommand(() -> setPosition(angle), this);
    }

    /**
     * @return A command that sets the elevator to a specific position and waits until it is at that position.
     * It will not finish until the elevator is at the target position.
     */
    public Command getPositionCommandAndWait(Distance position) {
        // @formatter:off
        return (
            // Set the elevator to the desired position
            getPositionCommand(position)
                // Wait until the elevator is at the target position
                .andThen(Commands.waitUntil(() -> isElevatorAtTarget()))
        );
        // @formatter:on
    }

    public Command getPositionCommandAndWait(SuperstructureConstants.Level level) {
        // @formatter:off
        return (
            getPositionCommand(level)
                .andThen(Commands.waitUntil(this::isElevatorAtTarget))
        );
        // @formatter:on
    }

    public Command getPositionCommandAndWaitNotNearer(SuperstructureConstants.Level level) {
        // @formatter:off
        return (
            getPositionCommand(level)
                .andThen(Commands.waitUntil(() -> inputs.isAtTargetNotNearer))
        );
        // @formatter:on
    }

    public Command getPositionCommandAndWait(Angle angle) {
        // @formatter:off
        return (
            getPositionCommand(angle)
                .andThen(Commands.waitUntil(this::isElevatorAtTarget))
        );
        // @formatter:on
    }

    /**
     * @return The current position of the 1st stage of the elevator.
     */
    @AutoLogOutput(key = "ComponentPositions/Elevator/Stage1")
    public Pose3d getStage1Pose() {
        return new Pose3d(
            0.0,
            0.0,
            // MathUtil.clamp(
            //     inputs.height - ElevatorConstants.Position.STAGE_1_HEIGHT.in(Meters),
            //     0,
            //     ElevatorConstants.Position.STAGE_1_MAX_HEIGHT.in(Meters)
            // ),
            (inputs.height / ElevatorConstants.Position.MAX_HEIGHT.in(Meters)) *
            ElevatorConstants.Position.STAGE_1_MAX_HEIGHT.in(Meters),
            new Rotation3d()
        );
    }

    /**
     * @return The current position of the 2nd stage of the elevator.
     */
    @AutoLogOutput(key = "ComponentPositions/Elevator/Stage2")
    public Pose3d getStage2Pose() {
        return new Pose3d(0.0, 0.0, inputs.height, new Rotation3d());
    }
}
