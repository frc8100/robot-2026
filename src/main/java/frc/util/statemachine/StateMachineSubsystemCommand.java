package frc.util.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that executes actions based on the current state of a StateMachine.
 */
public class StateMachineSubsystemCommand<TStateType extends Enum<TStateType>, TPayload> extends Command {

    private final StateMachine<TStateType, TPayload> stateMachine;

    public StateMachineSubsystemCommand(StateMachine<TStateType, TPayload> stateMachine, Subsystem... requirements) {
        this.stateMachine = stateMachine;
        addRequirements(requirements);
    }

    /**
     * Runs the periodic action associated with the current state of the state machine.
     */
    @Override
    public void execute() {
        stateMachine.runCurrentStatePeriodicAction();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
