package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.SwerveState;
import frc.robot.subsystems.vision.Vision;
import frc.util.statemachine.StateCycle;
import frc.util.statemachine.StateMachine;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

public class ButtonBindings {

    /**
     * A wrapper around GenericHID for creating button bindings.
     * Use {@link #createBinding} to create button bindings.
     *
     * Example usage:
     * {@code
     * Controller controller1 = new Controller(0);
     * controller1.createBinding(XboxController.Button.A, (trigger) -> trigger.onTrue(new Command()));
     */
    public static class Controller extends GenericHID {

        /**
         * The directions of a POV button.
         * @see POVButton
         */
        public enum POVButtonDirection {
            UP(0),
            RIGHT(90),
            DOWN(180),
            LEFT(270);

            public final int angle;

            POVButtonDirection(int angle) {
                this.angle = angle;
            }
        }

        /**
         * Creates a controller on the specified port.
         * @param port - The port the controller is connected to as listed in the Driver Station.
         */
        public Controller(int port) {
            super(port);
        }

        public Trigger getJoystickButton(int buttonValue) {
            return new JoystickButton(this, buttonValue);
        }

        public Trigger getJoystickButton(XboxController.Button button) {
            return getJoystickButton(button.value);
        }

        public Trigger getPOVButton(int angle) {
            return new POVButton(this, angle);
        }

        public Trigger getPOVButton(POVButtonDirection direction) {
            return getPOVButton(direction.angle);
        }

        /**
         * @param button - The button to get the state of.
         * @return A BooleanSupplier that returns true when the button is pressed.
         */
        public BooleanSupplier getButtonSupplier(XboxController.Button button) {
            return () -> this.getRawButton(button.value);
        }

        /**
         * @param axis - The axis to get the value of.
         * @return A DoubleSupplier that returns the value of the axis between -1 and 1.
         */
        public DoubleSupplier getAxisSupplier(XboxController.Axis axis) {
            return () -> this.getRawAxis(axis.value);
        }

        /**
         * @param axis - The axis to get the value of.
         * @param invert - Whether to invert the value of the axis.
         * @return A DoubleConsumer that sets the value of the axis between -1 and 1.
         */
        public DoubleSupplier getAxisSupplier(XboxController.Axis axis, boolean invert) {
            return invert ? () -> (-1 * this.getRawAxis(axis.value)) : () -> this.getRawAxis(axis.value);
        }
    }

    // Subsystem references
    private final RobotActions autoRoutines;
    private final Swerve swerveSubsystem;
    private final Vision visionSubsystem;

    private final Controller driverController = new Controller(0);
    private final Controller operatorController = new Controller(1);

    public ButtonBindings(RobotActions autoRoutines) {
        this.autoRoutines = autoRoutines;

        // Get subsystems from AutoRoutines
        this.swerveSubsystem = autoRoutines.swerveSubsystem;
        this.visionSubsystem = autoRoutines.visionSubsystem;
    }

    /**
     * Creates button bindings for the robot.
     */
    public void configureButtonBindings() {
        // Driver controller bindings
        driverController
            .getJoystickButton(ControlConstants.mainDriveControls.zeroGyroButton)
            .onTrue(Commands.runOnce(swerveSubsystem::zeroGyro));
        // Toggle drive to coral station state
        // StateCycle<SwerveState, Supplier<Pose2d>> toggleDriveToCoralStation =
        //     swerveSubsystem.stateMachine.createStateCycleWithPayload(
        //         List.of(
        //             new StateMachine.StateWithPayload<>(SwerveState.FULL_DRIVER_CONTROL, null),
        //             new StateMachine.StateWithPayload<>(
        //                 SwerveState.DRIVE_TO_POSE_PATHFINDING,
        //                 RobotActions.FieldLocations.CORAL_STATION_1::getPose
        //             )
        //         ),
        //         StateCycle.StateCycleBehavior.RELY_ON_INDEX
        //     );

        // driverController
        //     .getJoystickButton(XboxController.Button.kA)
        //     .onTrue(Commands.runOnce(toggleDriveToCoralStation::scheduleNextState));
    }

    public void configureSimulationBindings() {
        // Spawn game pieces
        driverController
            .getJoystickButton(XboxController.Button.kB)
            .onTrue(
                Commands.runOnce(() ->
                    SimulatedArena.getInstance()
                        .addGamePiece(
                            new ReefscapeCoralOnField(
                                swerveSubsystem.getActualPose().plus(new Transform2d(1.0, 0.0, Rotation2d.kZero))
                            )
                        )
                )
            );
    }

    /**
     * Assigns default commands to subsystems.
     */
    public void assignDefaultCommands() {}
}
