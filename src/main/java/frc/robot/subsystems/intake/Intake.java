package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANIdConstants;
import frc.robot.subsystems.CANIdAlert;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem.
 */
public class Intake extends SubsystemBase {

    /**
     * Gets the position of the intake center given the robot pose.
     * @param robotPose - The robot pose.
     * @return The intake center position.
     */
    public static Translation2d getIntakeCenterPosition(Pose2d robotPose) {
        return robotPose
            .getTranslation()
            .plus(IntakeConstants.ROBOT_CENTER_TO_INTAKE_CENTER.getTranslation().rotateBy(robotPose.getRotation()));
    }

    /**
     * States for the intake state machine.
     * - In {@link #TRANSITION_DEPLOYING} and {@link #TRANSITION_RETRACTING}, the intake is in the process of deploying or retracting, and cannot transition to the opposite state until it finishes transitioning to the current state.
     */
    public enum IntakeState {
        /**
         * The intake is fully deployed.
         */
        DEPLOYED,

        /**
         * The intake is going from retracted to deployed.
         */
        TRANSITION_DEPLOYING,

        /**
         * The intake is going from deployed to retracted.
         */
        TRANSITION_RETRACTING,

        /**
         * The intake is fully retracted.
         */
        RETRACTED,
    }

    public final StateMachine<IntakeState, Object> stateMachine = new StateMachine<IntakeState, Object>(
        IntakeState.class,
        "Intake"
    )
        .withDefaultState(
            new StateMachineState<>(IntakeState.RETRACTED, "Retracted").withCanChangeCondition(
                previousState -> previousState == IntakeState.TRANSITION_RETRACTING
            )
        )
        .withState(
            new StateMachineState<>(IntakeState.DEPLOYED, "Deployed").withCanChangeCondition(
                previousState -> previousState == IntakeState.TRANSITION_DEPLOYING
            )
        )
        .withState(new StateMachineState<>(IntakeState.TRANSITION_DEPLOYING, "TransitionDeploying"))
        .withState(new StateMachineState<>(IntakeState.TRANSITION_RETRACTING, "TransitionRetracting"));

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // Alerts for disconnected motors
    private final CANIdAlert intakeDisconnectedAlert = new CANIdAlert(CANIdConstants.INTAKE_MOTOR_ID, "IntakeMotor");

    // private final CANIdAlert deployDisconnectedAlert = new CANIdAlert(CANIdConstants.DEPLOY_MOTOR_ID, "DeployMotor");

    public Intake(IntakeIO io) {
        this.io = io;

        // State machine bindings
        stateMachine.whileState(IntakeState.DEPLOYED, () -> {
            // If intake is ever "undeployed" while in the deployed state, deploy again
            if (inputs.measuredDeployState != IntakeIO.MeasuredDeployState.DEPLOYED) {
                stateMachine.scheduleStateChange(IntakeState.TRANSITION_DEPLOYING);
            }
        });
        stateMachine.whileState(IntakeState.RETRACTED, () -> {});

        stateMachine.whileState(IntakeState.TRANSITION_DEPLOYING, () -> {
            deploy();
            // If intake is deployed, transition to deployed state
            if (inputs.measuredDeployState == IntakeIO.MeasuredDeployState.DEPLOYED) {
                stateMachine.scheduleStateChange(IntakeState.DEPLOYED);
            }
        });
        stateMachine.whileState(IntakeState.TRANSITION_RETRACTING, () -> {
            retract();
            // If intake is retracted, transition to retracted state
            if (inputs.measuredDeployState == IntakeIO.MeasuredDeployState.RETRACTED) {
                stateMachine.scheduleStateChange(IntakeState.RETRACTED);
            }
        });

        setDefaultCommand(stateMachine.getRunnableCommand(this));
    }

    // temporary
    public Command runIntake(double speed) {
        return run(() -> io.runIntake(speed));
    }

    public Command runIntake(DoubleSupplier speedSupplier) {
        return run(() -> io.runIntake(speedSupplier.getAsDouble()));
    }

    public void deploy() {
        io.deploy();
    }

    public void retract() {
        io.retract();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Update alerts
        intakeDisconnectedAlert.updateConnectionStatus(inputs.intakeMotorConnected);
        // deployDisconnectedAlert.updateConnectionStatus(inputs.deployMotorConnected);
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
    }
}
