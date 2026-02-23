package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CANIdConstants;
import frc.robot.Constants;
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
    private final CANIdAlert deployDisconnectedAlert = new CANIdAlert(CANIdConstants.DEPLOY_MOTOR_ID, "DeployMotor");

    // Deploy state visualization
    private final LinearFilter deployStateFilter = LinearFilter.movingAverage(
        (int) ((1000.0 * Constants.LOOP_PERIOD_SECONDS) * IntakeConstants.SIMULATION_TIME_FOR_INTAKE_DEPLOY.in(Seconds))
    );
    private final MutAngle deployStateForVisualization = Radians.mutable(0.0);

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

    /**
     * @return A value from 0 to 1 representing the deploy state of the intake for visualization. Goes from 0 to 1 as the intake deploys, and from 1 to 0 as the intake retracts, with a delay to match the time it takes for the intake to deploy/retract in simulation.
     */
    public double getDeployStateForVisualization() {
        // return deployStateFilter.calculate(
        //     inputs.measuredDeployState == IntakeIO.MeasuredDeployState.DEPLOYED ? 1.0 : 0.0
        // );

        return MathUtil.inverseInterpolate(
            IntakeConstants.INTAKE_RETRACTED_ANGLE.baseUnitMagnitude(),
            IntakeConstants.INTAKE_DEPLOYED_ANGLE.baseUnitMagnitude(),
            inputs.deployMotorData.positionAngle.baseUnitMagnitude()
        );
    }

    public Angle getDeployAngle() {
        // return deployStateForVisualization.mut_replace(MathUtil.interpolate(
        //     IntakeConstants.INTAKE_RETRACTED_ANGLE.in(Radians),
        //     IntakeConstants.INTAKE_DEPLOYED_ANGLE.in(Radians),
        //     getDeployStateForVisualization()
        // ), Radians);

        return deployStateForVisualization.mut_replace(inputs.deployMotorData.positionAngle);
    }

    // temporary
    public Command runIntake() {
        // No requirement because also run state machine at same time (state machine does not require intake subsystem, so can run at same time as this command)
        return Commands.run(() -> io.runIntake(IntakeConstants.INTAKE_RUN_SPEED));
    }

    public Command stopIntake() {
        return Commands.run(() -> io.runIntake(0));
    }

    /**
     * @return A command that moves the intake deploy back until it hits the hard stop and then zeros the intake.
     */
    public Command calibrateIntakeDeploy() {
        /**
         * A trigger that is true when the intake deploy motor is stalled which indicates that the intake has hit the hard stop.
         */
        final Trigger calibrationCompleteTrigger = new Trigger(() ->
            inputs.deployMotorData.torqueCurrent.gte(Amps.of(10))
        ).debounce(0.1);

        return run(() -> io.runDeployDutyCycle(-0.1))
            .until(calibrationCompleteTrigger)
            .andThen(runOnce(() -> io.setDeployEncoderPosition(IntakeConstants.INTAKE_RETRACTED_ANGLE)));
    }

    /**
     * Deploys the intake.
     */
    public void deploy() {
        io.deploy();
    }

    /**
     * Retracts the intake.
     */
    public void retract() {
        io.retract();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Update alerts
        intakeDisconnectedAlert.updateConnectionStatus(inputs.intakeMotorConnected);
        deployDisconnectedAlert.updateConnectionStatus(inputs.deployMotorConnected);

        // Visualization
        Logger.recordOutput("Intake/VisualizationPose", getIntakePose());
    }

    /**
     * @return The pose of the intake relative to the robot.
     */
    public Transform3d getIntakePose() {
        return new Transform3d(Translation3d.kZero, new Rotation3d(Radians.zero(), getDeployAngle(), Radians.zero()));
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
    }
}
