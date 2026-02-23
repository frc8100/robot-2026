package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANIdConstants;
import frc.robot.subsystems.CANIdAlert;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

    public enum ClimbState {
        /**
         * The climb is idle and stowed.
         * Can transition to {@link #DEPLOYING}.
         */
        IDLE,

        /**
         * The climb is deploying from the stowed position to the "ready to climb" position.
         */
        DEPLOYING,

        /**
         * The climb is fully deployed and ready to climb.
         * Can transition to {@link #CLIMBING}.
         */
        DEPLOYED,

        /**
         * The climb is climbing.
         */
        CLIMBING,

        /**
         * The climb is holding its position.
         */
        HOLD_CLIMB,

        /**
         * The climb is descending.
         */
        DESCENDING,
    }

    public final StateMachine<ClimbState, Object> stateMachine = new StateMachine<ClimbState, Object>(
        ClimbState.class,
        "Climb"
    )
        .withDefaultState(new StateMachineState<>(ClimbState.IDLE, "Idle"))
        .withState(new StateMachineState<>(ClimbState.DEPLOYING, "Deploying").requirePreviousStateToBe(ClimbState.IDLE))
        .withState(
            new StateMachineState<>(ClimbState.DEPLOYED, "Deployed").requirePreviousStateToBe(ClimbState.DEPLOYING)
        )
        .withState(
            new StateMachineState<>(ClimbState.CLIMBING, "Climbing").requirePreviousStateToBe(ClimbState.DEPLOYING)
        )
        .withState(
            new StateMachineState<>(ClimbState.HOLD_CLIMB, "HoldClimb").requirePreviousStateToBe(ClimbState.CLIMBING)
        )
        .withState(
            new StateMachineState<>(ClimbState.DESCENDING, "Descending").requirePreviousStateToBe(ClimbState.CLIMBING)
        );

    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    // Alerts
    private final CANIdAlert leftDisconnectedAlert = new CANIdAlert(
        CANIdConstants.LEFT_CLIMB_MOTOR_ID,
        "LeftClimbMotor"
    );
    private final CANIdAlert rightDisconnectedAlert = new CANIdAlert(
        CANIdConstants.RIGHT_CLIMB_MOTOR_ID,
        "RightClimbMotor"
    );

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);

        // Update alerts
        leftDisconnectedAlert.updateConnectionStatus(inputs.leftClimbMotorConnected);
        rightDisconnectedAlert.updateConnectionStatus(inputs.rightClimbMotorConnected);
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
    }
}
