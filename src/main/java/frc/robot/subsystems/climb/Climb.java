package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;

public class Climb extends SubsystemBase {

    public enum ClimbState {
        IDLE,
        DEPLOYING,
        CLIMBING,
        DESCENDING,
    }

    public final StateMachine<ClimbState, Object> stateMachine = new StateMachine<ClimbState, Object>(
        ClimbState.class,
        "Climb"
    )
        .withDefaultState(new StateMachineState<>(ClimbState.IDLE, "Idle"))
        .withState(new StateMachineState<>(ClimbState.DEPLOYING, "Deploying").requirePreviousStateToBe(ClimbState.IDLE))
        .withState(
            new StateMachineState<>(ClimbState.CLIMBING, "Climbing").requirePreviousStateToBe(ClimbState.DEPLOYING)
        )
        .withState(
            new StateMachineState<>(ClimbState.DESCENDING, "Descending").requirePreviousStateToBe(ClimbState.CLIMBING)
        );
}
