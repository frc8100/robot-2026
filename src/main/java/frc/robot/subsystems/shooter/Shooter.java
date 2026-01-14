package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem.
 */
public class Shooter extends SubsystemBase {

    public enum ShooterState {
        // /**
        //  * The intake is fully deployed.
        //  */
        // DEPLOYED,

        // /**
        //  * The intake is in transition between deployed and retracted.
        //  */
        // TRANSITION_DEPLOYING,

        // /**
        //  * The intake is in transition between retracted and deployed.
        //  */
        // TRANSITION_RETRACTING,

        // /**
        //  * The intake is fully retracted.
        //  */
        // RETRACTED,
    }

    public final StateMachine<ShooterState, Object> stateMachine = new StateMachine<ShooterState, Object>(
        ShooterState.class,
        "Shooter"
    );
    // .withDefaultState(new StateMachineState<>(ShooterState.RETRACTED, "Retracted"))
    // .withState(new StateMachineState<>(ShooterState.DEPLOYED, "Deployed"))
    // .withState(new StateMachineState<>(ShooterState.TRANSITION_DEPLOYING, "Transition Deploying"))
    // .withState(new StateMachineState<>(ShooterState.TRANSITION_RETRACTING, "Transition Retracting"));

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
