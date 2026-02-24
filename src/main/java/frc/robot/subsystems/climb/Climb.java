package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANIdConstants;
import frc.robot.subsystems.CANIdAlert;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

    public enum ClimbLocation {
        LEFT,
        RIGHT,
    }

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

        /**
         * The climb is retracting from the deployed position back to the stowed position.
         */
        RETRACTING,
    }

    public final StateMachine<ClimbState, Object> stateMachine = new StateMachine<ClimbState, Object>(
        ClimbState.class,
        "Climb"
    )
        .withDefaultState(
            new StateMachineState<>(ClimbState.IDLE, "Idle").requirePreviousStateToBe(ClimbState.RETRACTING)
        )
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
            new StateMachineState<>(ClimbState.DESCENDING, "Descending").requirePreviousStateToBeOneOf(
                ClimbState.CLIMBING,
                ClimbState.HOLD_CLIMB
            )
        )
        .withState(
            new StateMachineState<>(ClimbState.RETRACTING, "Retracting").requirePreviousStateToBeOneOf(
                ClimbState.DEPLOYED,
                ClimbState.DEPLOYING
            )
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

        // State machine bindings
        stateMachine.whileState(ClimbState.DEPLOYING, this::handleDeploying);
        stateMachine.whileState(ClimbState.DEPLOYED, this::handleDeployed);
        stateMachine.whileState(ClimbState.CLIMBING, this::handleClimbing);
        stateMachine.whileState(ClimbState.HOLD_CLIMB, this::handleHoldClimb);
        stateMachine.whileState(ClimbState.DESCENDING, this::handleDescending);

        setDefaultCommand(stateMachine.getRunnableCommand(this));
    }

    // State bindings
    private void handleDeploying() {
        io.setClimbTarget(ClimbConstants.DEPLOY_ANGLE);

        if (isAtTargetAngle(ClimbConstants.DEPLOY_ANGLE)) {
            stateMachine.scheduleStateChange(ClimbState.DEPLOYED);
        }
    }

    private void handleDeployed() {
        io.setClimbTarget(ClimbConstants.DEPLOY_ANGLE);

        if (!isAtTargetAngle(ClimbConstants.DEPLOY_ANGLE)) {
            stateMachine.scheduleStateChange(ClimbState.DEPLOYING);
        }
    }

    private void handleClimbing() {
        io.setClimbTarget(ClimbConstants.CLIMB_ANGLE);

        if (isAtTargetAngle(ClimbConstants.CLIMB_ANGLE)) {
            stateMachine.scheduleStateChange(ClimbState.HOLD_CLIMB);
        }
    }

    private void handleHoldClimb() {
        io.setClimbTarget(ClimbConstants.CLIMB_ANGLE);

        if (!isAtTargetAngle(ClimbConstants.CLIMB_ANGLE)) {
            stateMachine.scheduleStateChange(ClimbState.CLIMBING);
        }
    }

    private void handleDescending() {
        handleDeploying();
        // io.setClimbTarget(ClimbConstants.DEPLOY_ANGLE);

        // if (isAtTargetAngle(ClimbConstants.DEPLOY_ANGLE)) {
        //     stateMachine.scheduleStateChange(ClimbState.DEPLOYED);
        // }
    }

    public boolean isAtTargetAngle(Angle target) {
        return target.isNear(inputs.leftClimbMotorData.positionAngle, ClimbConstants.TOLERANCE);
    }

    /**
     * @return The robot transform based on the state of the climber.
     */
    public Rotation3d getRobotTransform() {
        // If climb is not climbing, no robot transform
        if (
            stateMachine.is(ClimbState.IDLE) ||
            stateMachine.is(ClimbState.DEPLOYING) ||
            stateMachine.is(ClimbState.DEPLOYED) ||
            stateMachine.is(ClimbState.RETRACTING)
        ) {
            return Rotation3d.kZero;
        }

        double climbImpartedRad =
            ClimbConstants.ANGLE_BEFORE_START_TO_LATCH.in(Radians) -
            inputs.leftClimbMotorData.positionAngle.in(Radians);

        return new Rotation3d(0.0, climbImpartedRad, 0.0);
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
    }
}
