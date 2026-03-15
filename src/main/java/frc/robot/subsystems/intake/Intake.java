package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CANIdConstants;
import frc.robot.Constants;
import frc.robot.subsystems.DeviceAlert;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
     * The measured deploy state of the intake based on the deploy motor encoder position.
     */
    public enum MeasuredDeployState {
        DEPLOYED,
        RETRACTED,
        TRANSITION,
    }

    /**
     * The direction the intake is moving.
     * Used when commanding duty cycle outputs.
     *
     * The angle of the intake is based on if you look at the robot such that the intake is on the left, a counter clockwise rotation is an increase in the reported angle.
     * Ex. when the intake angle is 90*, the intake is straight up/retracted. When the intake angle is 180*, the intake is pointing to the left/deployed.
     */
    public enum IntakeDeployDirection {
        /**
         * The intake rotates outwards/counterclockwise/positive.
         */
        DEPLOYING(1.0),

        /**
         * The intake rotates backwards/clockwise/negative.
         */
        RETRACTING(-1.0);

        public final double factor;

        private IntakeDeployDirection(double factor) {
            this.factor = factor;
        }
    }

    /**
     * States for the intake state machine. Mostly for the deploy state (intake rollers run independently).
     */
    public enum IntakeState {
        /**
         * The intake is fully deployed. No output is applied to the deploy motor.
         */
        DEPLOYED_RESTING,

        /**
         * The intake is fully retracted. No output is applied to the deploy motor.
         */
        RETRACTED_RESTING,

        /**
         * The deploy motor is running a closed loop controller to deploy the intake. Transitions to {@link #DEPLOYED_RESTING} when the intake is fully deployed.
         */
        TRANSITION_DEPLOYING,

        /**
         * The deploy motor is running a closed loop controller to retract the intake. Transitions to {@link #RETRACTED_RESTING} when the intake is fully retracted.
         */
        TRANSITION_RETRACTING,

        // TODO: determine if states below are needed

        /**
         * The intake deploy is running duty cycle based on the voltage controlled by the human operator.
         */
        TEST_VOLTAGE_CONTROL,

        /**
         * The intake deploy is running duty cycle to retract.
         * Automatically transitions to {@link #RETRACTED} when deploy has hit the hard stop.
         */
        CALIBRATE_RETRACT,

        TEST_SETPOINT_CHANGE,
    }

    public final StateMachine<IntakeState, Object> stateMachine = new StateMachine<IntakeState, Object>(
        IntakeState.class,
        "Intake"
    )
        .withDefaultState(new StateMachineState<>(IntakeState.RETRACTED_RESTING, "Retract"))
        .withState(new StateMachineState<>(IntakeState.DEPLOYED_RESTING, "Deploy"))
        .withState(new StateMachineState<>(IntakeState.TRANSITION_DEPLOYING, "TransitionDeploying"))
        .withState(new StateMachineState<>(IntakeState.TRANSITION_RETRACTING, "TransitionRetracting"))
        .withState(new StateMachineState<>(IntakeState.TEST_VOLTAGE_CONTROL, "TestVoltage"))
        .withState(new StateMachineState<>(IntakeState.CALIBRATE_RETRACT, "CalibrateRetract"))
        .withState(new StateMachineState<>(IntakeState.TEST_SETPOINT_CHANGE, "TestSetpointChange"));

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // Alerts for disconnected motors
    private final DeviceAlert intakeDisconnectedAlert = new DeviceAlert(CANIdConstants.ROLLER_MOTOR_ID, "IntakeMotor");
    private final DeviceAlert deployDisconnectedAlert = new DeviceAlert(CANIdConstants.DEPLOY_MOTOR_ID, "DeployMotor");

    // Deploy state visualization
    private final LinearFilter deployStateFilter = LinearFilter.movingAverage(
        (int) ((1000.0 * Constants.LOOP_PERIOD_SECONDS) * IntakeConstants.SIMULATION_TIME_FOR_INTAKE_DEPLOY.in(Seconds))
    );
    private final MutAngle deployStateForVisualization = Radians.mutable(0.0);

    private final SysIdRoutine intakeSysid;

    private final MutVoltage testVoltageOutput = Volts.mutable(0.0);

    /**
     * A trigger that is true when the intake deploy motor is stalled which indicates that the intake has hit the hard stop.
     */
    private final Trigger calibrationCompleteTrigger = new Trigger(() ->
        inputs.deployMotorData.torqueCurrent.gte(Amps.of(28))
    ).debounce(0.3);

    private final LoggedNetworkNumber deploySetpointTestDegrees = new LoggedNetworkNumber(
        "Intake/DeploySetpointTest",
        IntakeConstants.INTAKE_RETRACTED_ANGLE.in(Degrees)
    );
    private final MutAngle deploySetpointTestAngle = Degrees.mutable(0.0);
    private final MutAngle deploySetpointTestAngleOffset = Degrees.mutable(0.0);

    private boolean isRollersRunning = false;

    /**
     * Creates a new Intake subsystem.
     * @param io - The IO interface for the intake subsystem.
     */
    public Intake(IntakeIO io) {
        this.io = io;

        // State machine bindings
        stateMachine.whileState(IntakeState.DEPLOYED_RESTING, this::handleDeployRest);
        stateMachine.whileState(IntakeState.RETRACTED_RESTING, this::handleRetractRest);
        stateMachine.whileState(IntakeState.TRANSITION_DEPLOYING, this::handleTransitionDeploying);
        stateMachine.whileState(IntakeState.TRANSITION_RETRACTING, this::handleTransitionRetracting);
        stateMachine.whileState(IntakeState.CALIBRATE_RETRACT, this::handleCalibrateRetract);
        stateMachine.whileState(IntakeState.TEST_VOLTAGE_CONTROL, () -> io.runDeployVoltage(testVoltageOutput));
        stateMachine.whileState(IntakeState.TEST_SETPOINT_CHANGE, () -> {
            io.setDeploySetpoint(deploySetpointTestAngle.mut_replace(deploySetpointTestDegrees.get(), Degrees));
            Logger.recordOutput("Intake/DeploySetpointTest", deploySetpointTestDegrees);
        });

        StateMachine.onDriverStationDisable.onTrue(
            Commands.runOnce(() -> {
                isRollersRunning = false;
            })
        );

        stateMachine.onStateChange(IntakeState.CALIBRATE_RETRACT, this::onCalibrateDeploy);

        setDefaultCommand(stateMachine.getRunnableCommand(this));

        intakeSysid = new SysIdRoutine(
            new SysIdRoutine.Config(
                IntakeConstants.SYSID_RAMP_RATE,
                IntakeConstants.SYSID_MAX_VOLTAGE,
                IntakeConstants.SYSID_TEST_DURATION,
                state -> Logger.recordOutput("Intake/SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(io::runDeployVoltage, null, this)
        );
    }

    /**
     * @return The measured deploy state of the intake based on the deploy motor encoder position.
     */
    public MeasuredDeployState getMeasuredDeployState() {
        return getMeasuredDeployState(stateMachine.getCurrentState().enumType);
    }

    /**
     * @return Returns the measured deploy state of the intake based on the provided state.
     */
    public MeasuredDeployState getMeasuredDeployState(IntakeState currentState) {
        Angle currentTargetDeployAngle = Radians.zero();
        Angle targetTolerance = Radians.zero();
        MeasuredDeployState outputMeasuredDeployState = MeasuredDeployState.TRANSITION;

        // Determine target angle based on state
        if (currentState == IntakeState.DEPLOYED_RESTING || currentState == IntakeState.TRANSITION_DEPLOYING) {
            currentTargetDeployAngle = IntakeConstants.INTAKE_DEPLOYED_ANGLE;
            targetTolerance = IntakeConstants.DEPLOY_TARGET_TOLERANCE;
            outputMeasuredDeployState = MeasuredDeployState.DEPLOYED;
        } else if (currentState == IntakeState.RETRACTED_RESTING || currentState == IntakeState.TRANSITION_RETRACTING) {
            // currentTargetDeployAngle = IntakeConstants.INTAKE_RETRACTED_ANGLE;
            currentTargetDeployAngle = IntakeConstants.INTAKE_RETRACTED_ANGLE_SETPOINT;
            targetTolerance = IntakeConstants.RETRACT_TARGET_TOLERANCE;
            outputMeasuredDeployState = MeasuredDeployState.RETRACTED;
        } else {
            return MeasuredDeployState.TRANSITION;
        }

        // Compare
        if (inputs.deployMotorData.positionAngle.isNear(currentTargetDeployAngle, targetTolerance)) {
            return outputMeasuredDeployState;
        } else {
            return MeasuredDeployState.TRANSITION;
        }
    }

    /**
     * Runs the intake deploy motor at a given duty cycle in a given direction. Used for manual control of the intake deploy for testing and as a backup to the state machine.
     * @param direction - The direction to run the intake deploy motor.
     * @param speed - The duty cycle to run the intake deploy motor at from 0 to 1. The direction parameter determines if this is positive or negative duty cycle.
     */
    public void runDeployDutyCycle(IntakeDeployDirection direction, double speed) {
        io.runDeployDutyCycle(direction.factor * MathUtil.clamp(speed, 0, 1));
    }

    /**
     * @return The angle to use for visualization of the intake deploy state.
     */
    public Angle getDeployAngle() {
        return deployStateForVisualization.mut_replace(inputs.deployMotorData.positionAngle);
    }

    /**
     * @return A command to run the roller.
     * No requirement because also run state machine at same time (state machine does not require intake subsystem, so can run at same time as this command)
     */
    // public Command runRollerCommand() {
    //     return Commands.run(() -> io.runRollerDutyCycle(IntakeConstants.INTAKE_RUN_SPEED));
    // }

    /**
     * @return A command to stop the roller.
     */
    // public Command stopRollerCommand() {
    //     return Commands.run(() -> io.runRollerDutyCycle(0));
    // }

    public void toggleRollers() {
        isRollersRunning = !isRollersRunning;
    }

    public void setRollers(boolean set) {
        isRollersRunning = set;
    }

    /**
     * @return A command to run the intake deploy motor at a given duty cycle in a given direction.
     * @param direction - The direction to run the intake deploy motor.
     * @param speed - The duty cycle to run the intake deploy motor at from 0 to 1. The direction parameter determines if this is positive or negative duty cycle.
     */
    public Command runDeployDutyCycleCommand(IntakeDeployDirection direction, double speed) {
        return run(() -> runDeployDutyCycle(direction, speed));
    }

    /**
     * @return A command to stop the intake deploy motor.
     */
    public Command stopDeployDutyCycleCommand() {
        return run(() -> runDeployDutyCycle(IntakeDeployDirection.DEPLOYING, 0));
    }

    /**
     * Called when transitioning to the {@link IntakeState#CALIBRATE_RETRACT} state.
     * Removes the soft limits on the deploy motor to allow the intake to hit the hard stop and calibrate the encoder.
     */
    public void onCalibrateDeploy() {
        io.setSoftLimits(false);
    }

    /**
     * Called periodically while in the {@link IntakeState#CALIBRATE_RETRACT} state.
     */
    public void handleCalibrateRetract() {
        runDeployDutyCycle(IntakeDeployDirection.RETRACTING, 0.2);

        if (calibrationCompleteTrigger.getAsBoolean()) {
            io.setDeployEncoderPosition(IntakeConstants.INTAKE_RETRACTED_ANGLE);
            stateMachine.scheduleStateChange(IntakeState.RETRACTED_RESTING);
        }
    }

    /**
     * @return A command to do all 4 sysid routines.
     * Start with deploy retracted.
     */
    public Command sysid() {
        final Trigger sysidCompleteForwardTrigger = new Trigger(
            () -> getMeasuredDeployState(IntakeState.DEPLOYED_RESTING) == MeasuredDeployState.DEPLOYED
        ).debounce(0.2);

        final Trigger sysidCompleteReverseTrigger = new Trigger(
            () -> getMeasuredDeployState(IntakeState.RETRACTED_RESTING) == MeasuredDeployState.RETRACTED
        ).debounce(0.2);

        Command resetForward = Commands.runOnce(() -> io.setDeployEncoderPosition(IntakeConstants.INTAKE_DEPLOYED_ANGLE)
        );
        Command resetReverse = Commands.runOnce(() ->
            io.setDeployEncoderPosition(IntakeConstants.INTAKE_RETRACTED_ANGLE)
        );

        return new SequentialCommandGroup(
            intakeSysid
                .quasistatic(SysIdRoutine.Direction.kForward)
                .until(sysidCompleteForwardTrigger.or(calibrationCompleteTrigger)),
            Commands.runOnce(() -> io.setDeployEncoderPosition(IntakeConstants.INTAKE_DEPLOYED_ANGLE)),
            Commands.waitSeconds(0.5),
            intakeSysid
                .quasistatic(SysIdRoutine.Direction.kReverse)
                .until(sysidCompleteReverseTrigger.or(calibrationCompleteTrigger)),
            Commands.runOnce(() -> io.setDeployEncoderPosition(IntakeConstants.INTAKE_RETRACTED_ANGLE)),
            Commands.waitSeconds(0.5),
            intakeSysid
                .dynamic(SysIdRoutine.Direction.kForward)
                .until(sysidCompleteForwardTrigger.or(calibrationCompleteTrigger)),
            Commands.runOnce(() -> io.setDeployEncoderPosition(IntakeConstants.INTAKE_DEPLOYED_ANGLE)),
            Commands.waitSeconds(0.5),
            intakeSysid
                .dynamic(SysIdRoutine.Direction.kReverse)
                .until(sysidCompleteReverseTrigger.or(calibrationCompleteTrigger)),
            Commands.runOnce(() -> io.setDeployEncoderPosition(IntakeConstants.INTAKE_RETRACTED_ANGLE))
        );
        // return new SequentialCommandGroup(
        // return intakeSysid
        //     .quasistatic(SysIdRoutine.Direction.kForward)
        //     .until(sysidCompleteForwardTrigger.or(calibrationCompleteTrigger))
        //     .andThen(resetPositionForward)
        //     .andThen(Commands.waitSeconds(0.5))
        //     .andThen(
        //         intakeSysid
        //             .quasistatic(SysIdRoutine.Direction.kReverse)
        //             .until(sysidCompleteReverseTrigger.or(calibrationCompleteTrigger))
        //     )
        //     .andThen(Commands.waitSeconds(0.5))
        //     .andThen(
        //         intakeSysid
        //             .dynamic(SysIdRoutine.Direction.kForward)
        //             .until(sysidCompleteReverseTrigger.or(calibrationCompleteTrigger))
        //     )
        //     .andThen(resetPositionReverse)
        //     .andThen(Commands.waitSeconds(0.5))
        //     .andThen(
        //         intakeSysid
        //             .dynamic(SysIdRoutine.Direction.kReverse)
        //             .until(sysidCompleteReverseTrigger.or(calibrationCompleteTrigger))
        //             .andThen(resetPositionForward)
        //     );
        // ,
        // resetPositionReverse,
        // Commands.waitSeconds(0.5),
        // intakeSysid
        //     .dynamic(SysIdRoutine.Direction.kForward)
        //     .until(sysidCompleteForwardTrigger.or(calibrationCompleteTrigger)),
        // resetPositionForward,
        // Commands.waitSeconds(0.5),
        // intakeSysid
        //     .dynamic(SysIdRoutine.Direction.kReverse)
        //     .until(sysidCompleteReverseTrigger.or(calibrationCompleteTrigger)),
        // resetPositionReverse
        // );
        // return Commands.none();
    }

    public Command changeAngleOffset(Angle change) {
        return Commands.runOnce(() -> deploySetpointTestAngleOffset.mut_plus(change));
    }

    public Command setAngleOffset(Angle set) {
        return Commands.runOnce(() -> deploySetpointTestAngleOffset.mut_replace(set));
    }

    /**
     * Deploys the intake.
     * Called periodically while in the {@link IntakeState#DEPLOYED_RESTING} state by the state machine.
     */
    private void handleDeployRest() {
        // io.setDeploySetpoint(IntakeConstants.INTAKE_DEPLOYED_ANGLE, deploySetpointTestAngleOffset);

        io.stopDeploy();

        if (getMeasuredDeployState() != MeasuredDeployState.DEPLOYED) {
            stateMachine.scheduleStateChange(IntakeState.TRANSITION_DEPLOYING);
            // TODO: this has a 1 frame lag
        }
    }

    /**
     * Retracts the intake.
     * Called periodically while in the {@link IntakeState#RETRACTED_RESTING} state by the state machine.
     */
    private void handleRetractRest() {
        // io.setDeploySetpoint(IntakeConstants.INTAKE_RETRACTED_ANGLE);

        runDeployDutyCycle(IntakeDeployDirection.RETRACTING, 0.08);

        if (getMeasuredDeployState() != MeasuredDeployState.RETRACTED) {
            stateMachine.scheduleStateChange(IntakeState.TRANSITION_RETRACTING);
        }
    }

    private void handleTransitionDeploying() {
        // Set the setpoint slightly below the deployed angle so that the motion profile decelerates as it approaches the deployed position, which should help prevent slamming into the hard stop
        io.setDeploySetpoint(IntakeConstants.INTAKE_DEPLOYED_ANGLE, deploySetpointTestAngleOffset);

        if (getMeasuredDeployState() == MeasuredDeployState.DEPLOYED || calibrationCompleteTrigger.getAsBoolean()) {
            stateMachine.scheduleStateChange(IntakeState.DEPLOYED_RESTING);
        }
    }

    private void handleTransitionRetracting() {
        io.setDeploySetpoint(IntakeConstants.INTAKE_RETRACTED_ANGLE_SETPOINT, deploySetpointTestAngleOffset);

        if (getMeasuredDeployState() == MeasuredDeployState.RETRACTED) {
            stateMachine.scheduleStateChange(IntakeState.RETRACTED_RESTING);
        }
    }

    public void changeTestOutVoltage(Voltage change) {
        testVoltageOutput.mut_plus(change);
    }

    public void setTestOutVoltage(Voltage set) {
        testVoltageOutput.mut_replace(set);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Update alerts
        intakeDisconnectedAlert.updateConnectionStatus(inputs.rollerMotorConnected);
        deployDisconnectedAlert.updateConnectionStatus(inputs.deployMotorConnected);

        if (isRollersRunning) {
            io.runRollerDutyCycle(IntakeConstants.INTAKE_RUN_SPEED);
        } else {
            io.runRollerDutyCycle(0.0);
        }

        // Visualization
        Logger.recordOutput("Intake/TestVoltOut", testVoltageOutput);
        Logger.recordOutput("Intake/VisualizationPose", getIntakePose());
    }

    /**
     * @return The pose of the intake relative to the robot.
     */
    public Transform3d getIntakePose() {
        return new Transform3d(
            IntakeConstants.CENTER_TO_INTAKE_PIVOT,
            new Rotation3d(Radians.zero(), getDeployAngle(), Radians.zero())
        );
    }

    @Override
    public void simulationPeriodic() {
        io.simIterate();
    }
}
