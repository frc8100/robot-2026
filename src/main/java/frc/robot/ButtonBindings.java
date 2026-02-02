package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.SwervePayload;
import frc.robot.subsystems.swerve.Swerve.SwerveState;
import frc.robot.subsystems.vision.Vision;
import frc.util.statemachine.StateCycle;
import frc.util.statemachine.StateMachine;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

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

        // Convenience methods for common usages
        public Trigger getButtonTrigger(XboxController.Button button) {
            return getJoystickButton(button);
        }

        public Trigger getButtonTrigger(POVButtonDirection direction) {
            return getPOVButton(direction);
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
    private final Intake intakeSubsystem;
    private final Shooter shooterSubsystem;

    public static final Controller driverController = new Controller(ControlConstants.DRIVER_CONTROLLER_PORT);
    private final Controller operatorController = new Controller(ControlConstants.OPERATOR_CONTROLLER_PORT);

    public ButtonBindings(RobotActions autoRoutines) {
        this.autoRoutines = autoRoutines;

        // Get subsystems from AutoRoutines
        this.swerveSubsystem = autoRoutines.swerveSubsystem;
        this.visionSubsystem = autoRoutines.visionSubsystem;
        this.intakeSubsystem = autoRoutines.intakeSubsystem;
        this.shooterSubsystem = autoRoutines.shooterSubsystem;
    }

    /**
     * Creates button bindings for the robot.
     */
    public void configureButtonBindings() {
        // Driver controller bindings
        driverController
            .getButtonTrigger(ControlConstants.mainDriveControls.zeroYawOffsetButton)
            .onTrue(Commands.runOnce(swerveSubsystem::zeroYawOffset));

        // Auto-aim toggle
        StateCycle<SwerveState, SwervePayload> toggleAutoAim = swerveSubsystem.stateMachine.createStateCycleWithPayload(
            List.of(
                new StateMachine.StateWithPayload<>(SwerveState.FULL_DRIVER_CONTROL, null),
                new StateMachine.StateWithPayload<>(SwerveState.AUTO_AIM, RobotActions.POINT_TO_HUB_PAYLOAD)
            ),
            StateCycle.StateCycleBehavior.RELY_ON_INDEX
        );
        // Auto intake toggle
        StateCycle<SwerveState, SwervePayload> toggleAutoIntake =
            swerveSubsystem.stateMachine.createStateCycleWithPayload(
                List.of(
                    new StateMachine.StateWithPayload<>(SwerveState.FULL_DRIVER_CONTROL, null),
                    new StateMachine.StateWithPayload<>(
                        SwerveState.DRIVE_TO_POSE_PID,
                        RobotActions.getIntakePayload(autoRoutines)
                    )
                ),
                StateCycle.StateCycleBehavior.RELY_ON_INDEX
            );

        // In the scenario where: 1. swerve is in FULL_DRIVER_CONTROL 2. driver presses X to go to AUTO_AIM
        // 3. driver presses DOWN to go to AUTO_INTAKE, 4. when driver presses X again, it will go back to AUTO_AIM
        driverController
            .getButtonTrigger(ControlConstants.toggleAutoAimToHub)
            .onTrue(
                Commands.runOnce(() -> {
                    toggleAutoAim.scheduleNextState();
                    // Reset the other toggle to make pressing the other toggle go to the corresponding state (instead of having to press twice)
                    toggleAutoIntake.reset();
                })
            );

        driverController
            .getButtonTrigger(ControlConstants.toggleAutoDriveIntake)
            .onTrue(
                Commands.runOnce(() -> {
                    toggleAutoIntake.scheduleNextState();
                    toggleAutoAim.reset();
                })
            );

        // Temporary shooter test button
        driverController
            .getButtonTrigger(ControlConstants.toggleShoot)
            .onTrue(Commands.runOnce(shooterSubsystem::testShoot));

        driverController
            .getButtonTrigger(ControlConstants.toggleAutoDriveIntake)
            .whileTrue(intakeSubsystem.runIntake(0.2));
    }

    /**
     * Creates button bindings for simulation-specific actions.
     */
    public void configureSimulationBindings() {
        // Spawn game pieces
        driverController
            .getButtonTrigger(XboxController.Button.kB)
            .onTrue(
                Commands.runOnce(() ->
                    SimulatedArena.getInstance()
                        .addGamePiece(
                            new RebuiltFuelOnField(
                                swerveSubsystem
                                    .getActualPose()
                                    .plus(new Transform2d(1.0, 0.0, Rotation2d.kZero))
                                    .getTranslation()
                            )
                        )
                )
            );

        // Spawn all fuel
        operatorController
            .getButtonTrigger(ControlConstants.spawnAllFuelButton)
            .onTrue(Commands.runOnce(() -> SimulatedArena.getInstance().placeGamePiecesOnField()));

        // Clear all game pieces
        operatorController
            .getButtonTrigger(ControlConstants.clearFuelButton)
            .onTrue(Commands.runOnce(() -> SimulatedArena.getInstance().clearGamePieces()));
    }

    /**
     * Assigns default commands to subsystems.
     */
    public void assignDefaultCommands() {
        // Currently unused
    }
}
