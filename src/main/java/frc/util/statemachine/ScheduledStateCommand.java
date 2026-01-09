package frc.util.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ScheduledStateCommand<TStateEnum extends Enum<TStateEnum>> extends Command {

    private final StateMachine<TStateEnum, ?> stateMachine;

    private TStateEnum stateToWaitFor = null;

    public ScheduledStateCommand(StateMachine<TStateEnum, ?> stateMachine) {
        this.stateMachine = stateMachine;
    }

    public void setStateToWaitFor(TStateEnum newState) {
        stateToWaitFor = newState;

        CommandScheduler.getInstance().removeComposedCommand(this);
    }

    @Override
    public void initialize() {}

    @Override
    public boolean isFinished() {
        return !stateMachine.isAnyStateScheduled() && stateMachine.getCurrentState().enumType == stateToWaitFor;
    }
}
