package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem.
 */
public class Intake extends SubsystemBase {

    public enum IntakeState {
        /**
         * The intake is fully deployed.
         */
        DEPLOYED,

        /**
         * The intake is in transition between deployed and retracted.
         */
        TRANSITION_DEPLOYING,

        /**
         * The intake is in transition between retracted and deployed.
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
        .withDefaultState(new StateMachineState<>(IntakeState.RETRACTED, "Retracted"))
        .withState(new StateMachineState<>(IntakeState.DEPLOYED, "Deployed"))
        .withState(new StateMachineState<>(IntakeState.TRANSITION_DEPLOYING, "Transition Deploying"))
        .withState(new StateMachineState<>(IntakeState.TRANSITION_RETRACTING, "Transition Retracting"));

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
