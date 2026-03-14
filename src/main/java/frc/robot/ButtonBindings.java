package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSpeeds;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.SwervePayload;
import frc.robot.subsystems.swerve.Swerve.SwerveState;
import frc.robot.subsystems.vision.Vision;
import frc.util.FuelSim;
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

        public Trigger getButtonTrigger(XboxController.Axis axis, double threshold) {
            return new Trigger(() -> Math.abs(super.getRawAxis(axis.value)) > threshold);
        }

        public Trigger getButtonTrigger(XboxController.Axis axis) {
            return getButtonTrigger(axis, 0.2);
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
    private final Climb climbSubsystem;

    public static final Controller driverController = new Controller(ControlConstants.DRIVER_CONTROLLER_PORT);
    public static final Controller operatorController = new Controller(ControlConstants.OPERATOR_CONTROLLER_PORT);

    public ButtonBindings(RobotActions autoRoutines) {
        this.autoRoutines = autoRoutines;

        // Get subsystems from AutoRoutines
        this.swerveSubsystem = autoRoutines.swerveSubsystem;
        this.visionSubsystem = autoRoutines.visionSubsystem;
        this.intakeSubsystem = autoRoutines.intakeSubsystem;
        this.shooterSubsystem = autoRoutines.shooterSubsystem;
        this.climbSubsystem = autoRoutines.climbSubsystem;
    }

    /**
     * Creates button bindings for the robot.
     */
    public void configureButtonBindings() {
        // Driver controller bindings
        operatorController
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

        // driverController
        //     .getButtonTrigger(ControlConstants.toggleAutoDriveIntake)
        //     .onTrue(
        //         Commands.runOnce(() -> {
        //             toggleAutoIntake.scheduleNextState();
        //             toggleAutoAim.reset();
        //         })
        //     );

        // Temporary shooter test button
        driverController
            .getButtonTrigger(ControlConstants.toggleShoot)
            .onTrue(
                Commands.runOnce(() ->
                    shooterSubsystem.stateMachine.scheduleStateChange(ShooterState.AUTO_TARGET_SHOOTING)
                )
            )
            .onFalse(Commands.runOnce(() -> shooterSubsystem.stateMachine.scheduleStateChange(ShooterState.IDLE)));

        operatorController
            .getButtonTrigger(XboxController.Button.kY)
            .whileTrue(shooterSubsystem.setspeedShoot(ShooterSpeeds.HIGHEST));

        // operatorController
        //     .getButtonTrigger(XboxController.Button.kRightBumper)
        //     .whileTrue(shooterSubsystem.runIndexer());

        operatorController
            .getButtonTrigger(ControlConstants.runIntakeRollers)
            // .whileTrue(intakeSubsystem.runRollerCommand())
            // .whileFalse(intakeSubsystem.stopRollerCommand());
            .onTrue(Commands.runOnce(intakeSubsystem::toggleRollers));

        // Intake deploy/retract toggle
        operatorController
            .getButtonTrigger(ControlConstants.toggleIntakeDeploy)
            .onTrue(
                Commands.runOnce(() -> {
                    // If the intake is currently retracted or retracting, deploy it. Otherwise, retract it.
                    if (
                        intakeSubsystem.stateMachine.is(IntakeState.RETRACTED_RESTING) ||
                        intakeSubsystem.stateMachine.is(IntakeState.TRANSITION_RETRACTING)
                    ) {
                        intakeSubsystem.stateMachine.scheduleStateChange(IntakeState.TRANSITION_DEPLOYING);
                    } else {
                        intakeSubsystem.stateMachine.scheduleStateChange(IntakeState.TRANSITION_RETRACTING);
                    }
                })
            );

        // Debug
        // operatorController
        //     .getButtonTrigger(ControlConstants.toggleIntakeDeployReverseTest)
        //     // higher to overcome gravity
        //     .whileTrue(intakeSubsystem.runDeployDutyCycleCommand(Intake.IntakeDeployDirection.RETRACTING, 0.2))
        //     .onFalse(intakeSubsystem.stopDeployDutyCycleCommand());

        // TODO: temporary voltage control
        // operatorController
        //     .getButtonTrigger(XboxController.Button.kB)
        //     .onTrue(
        //         Commands.runOnce(() ->
        //             // intakeSubsystem.stateMachine.scheduleStateChange(Intake.IntakeState.TEST_SETPOINT_CHANGE)
        //             shooterSubsystem.stateMachine.scheduleStateChange(Shooter.ShooterState.TEST_VOLTAGE_CONTROL)
        //         ).ignoringDisable(true)
        //     );
        // operatorController
        //     .getButtonTrigger(XboxController.Button.kX)
        //     .onTrue(
        //         Commands.runOnce(() ->
        //             intakeSubsystem.stateMachine.scheduleStateChange(Intake.IntakeState.TEST_VOLTAGE_CONTROL)
        //         ).ignoringDisable(true)
        //     );

        operatorController
            .getButtonTrigger(XboxController.Button.kX)
            .onTrue(
                Commands.runOnce(() ->
                    intakeSubsystem.stateMachine.scheduleStateChange(Intake.IntakeState.CALIBRATE_RETRACT)
                ).ignoringDisable(true)
            )
            .onFalse(
                Commands.runOnce(() ->
                    intakeSubsystem.stateMachine.scheduleStateChange(Intake.IntakeState.RETRACTED_RESTING)
                ).ignoringDisable(true)
            );

        // operatorController
        //     .getButtonTrigger(XboxController.Button.kY)
        //     .whileTrue(shooterSubsystem.runShooterDutyCycle(Volts.of(3)))
        //     .onFalse(shooterSubsystem.runShooterDutyCycle(Volts.zero()));

        // RPM offset
        // operatorController
        //     .getButtonTrigger(ControlConstants.increaseRPMOffset)
        //     .onTrue(shooterSubsystem.changeShooterRPMOffset(10.0));
        // operatorController
        //     .getButtonTrigger(ControlConstants.decreaseRPMOffset)
        //     .onTrue(shooterSubsystem.changeShooterRPMOffset(-10.0));

        Angle changeBy = Degrees.of(5);

        operatorController
            .getButtonTrigger(ButtonBindings.Controller.POVButtonDirection.UP)
            .onTrue(intakeSubsystem.changeAngleOffset(changeBy.unaryMinus()));

        operatorController
            .getButtonTrigger(ButtonBindings.Controller.POVButtonDirection.DOWN)
            .onTrue(intakeSubsystem.changeAngleOffset(changeBy));

        operatorController
            .getButtonTrigger(ButtonBindings.Controller.POVButtonDirection.RIGHT)
            .onTrue(intakeSubsystem.setAngleOffset(Degrees.zero()));
        // operatorController
        //     .getButtonTrigger(XboxController.Button.kRightBumper)
        //     .onTrue(
        //         Commands.runOnce(() ->
        //             shooterSubsystem.stateMachine.scheduleStateChange(ShooterState.MANUAL_TARGET_SHOOTING)
        //         )
        //     )
        //     .onFalse(Commands.runOnce(() -> shooterSubsystem.stateMachine.scheduleStateChange(ShooterState.IDLE)));
        // final Voltage incrementVoltage = Volts.of(0.5);
        // final Voltage fineIncrementVoltage = Volts.of(0.01);
        // final Voltage decrementVoltage = incrementVoltage.times(-1);
        // final Voltage fineDecrementVoltage = fineIncrementVoltage.times(-1);

        // operatorController
        //     .getButtonTrigger(ButtonBindings.Controller.POVButtonDirection.UP)
        //     .onTrue(
        //         Commands.runOnce(() -> shooterSubsystem.changeTestOutVoltageShooter(incrementVoltage)).ignoringDisable(
        //             true
        //         )
        //     );

        // operatorController
        //     .getButtonTrigger(ButtonBindings.Controller.POVButtonDirection.DOWN)
        //     .onTrue(
        //         Commands.runOnce(() -> shooterSubsystem.changeTestOutVoltageShooter(decrementVoltage)).ignoringDisable(
        //             true
        //         )
        //     );

        // operatorController
        //     .getButtonTrigger(ButtonBindings.Controller.POVButtonDirection.RIGHT)
        //     .onTrue(
        //         Commands.runOnce(() -> shooterSubsystem.changeTestOutVoltageIndexer(incrementVoltage)).ignoringDisable(
        //             true
        //         )
        //     );

        // operatorController
        //     .getButtonTrigger(ButtonBindings.Controller.POVButtonDirection.LEFT)
        //     .onTrue(
        //         Commands.runOnce(() -> shooterSubsystem.changeTestOutVoltageIndexer(decrementVoltage)).ignoringDisable(
        //             true
        //         )
        //     );

        // operatorController
        //     .getButtonTrigger(XboxController.Button.kA)
        //     .onTrue(
        //         Commands.runOnce(() -> {
        //             shooterSubsystem.setTestOutVoltageShooter(Volts.zero());
        //             shooterSubsystem.setTestOutVoltageIndexer(Volts.zero());
        //         }).ignoringDisable(true)
        //     );
        // TODO: Climb deploy/retract toggle
        // StateCycle<Climb.ClimbState, Object> toggleClimbDeploy =
        //     climbSubsystem.stateMachine.createStateCycleWithPayload(
        //         List.of(
        //             new StateMachine.StateWithPayload<>(Climb.ClimbState.RETRACTING, null),
        //             new StateMachine.StateWithPayload<>(Climb.ClimbState.DEPLOYING, null)
        //         ),
        //         StateCycle.StateCycleBehavior.RELY_ON_INDEX
        //     );

        // StateCycle<Climb.ClimbState, Object> toggleClimbClimb = climbSubsystem.stateMachine.createStateCycleWithPayload(
        //     List.of(
        //         new StateMachine.StateWithPayload<>(Climb.ClimbState.DESCENDING, null),
        //         new StateMachine.StateWithPayload<>(Climb.ClimbState.CLIMBING, null)
        //     ),
        //     StateCycle.StateCycleBehavior.RELY_ON_INDEX
        // );
        // driverController
        //     .getButtonTrigger(ControlConstants.deployClimbButton)
        //     // .onTrue(Commands.runOnce(toggleClimbDeploy::scheduleNextState));
        //     .onTrue(
        //         Commands.runOnce(() -> {
        //             System.out.println("Toggling climb deploy/retract");
        //             toggleClimbDeploy.scheduleNextState();
        //         })
        //     );

        // driverController
        //     .getButtonTrigger(ControlConstants.climbButton)
        //     .onTrue(Commands.runOnce(toggleClimbClimb::scheduleNextState));
    }

    /**
     * Creates button bindings for simulation-specific actions.
     */
    public void configureSimulationBindings() {
        // Spawn game pieces
        operatorController
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
            // .onTrue(Commands.runOnce(() -> SimulatedArena.getInstance().placeGamePiecesOnField()));
            .onTrue(Commands.runOnce(() -> FuelSim.getInstance().spawnStartingFuel()));

        // Clear all game pieces
        operatorController
            .getButtonTrigger(ControlConstants.clearFuelButton)
            .onTrue(Commands.runOnce(() -> FuelSim.getInstance().clearFuel()));
    }

    /**
     * Assigns default commands to subsystems.
     */
    public void assignDefaultCommands() {
        // Currently unused
    }
}
