package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
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
import frc.robot.subsystems.CANIdAlert;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachineState;
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
         * The intake is fully deployed.
         */
        DEPLOYED,

        /**
         * The intake is fully retracted.
         */
        RETRACTED,

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
    }

    public final StateMachine<IntakeState, Object> stateMachine = new StateMachine<IntakeState, Object>(
        IntakeState.class,
        "Intake"
    )
        .withDefaultState(new StateMachineState<>(IntakeState.RETRACTED, "Retract"))
        .withState(new StateMachineState<>(IntakeState.DEPLOYED, "Deploy"))
        .withState(new StateMachineState<>(IntakeState.TEST_VOLTAGE_CONTROL, "TestVoltage"))
        .withState(new StateMachineState<>(IntakeState.CALIBRATE_RETRACT, "CalibrateRetract"));

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // Alerts for disconnected motors
    private final CANIdAlert intakeDisconnectedAlert = new CANIdAlert(CANIdConstants.ROLLER_MOTOR_ID, "IntakeMotor");
    private final CANIdAlert deployDisconnectedAlert = new CANIdAlert(CANIdConstants.DEPLOY_MOTOR_ID, "DeployMotor");

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
        inputs.deployMotorData.torqueCurrent.gte(Amps.of(22))
    ).debounce(0.25);

    /**
     * Creates a new Intake subsystem.
     * @param io - The IO interface for the intake subsystem.
     */
    public Intake(IntakeIO io) {
        this.io = io;

        // State machine bindings
        stateMachine.whileState(IntakeState.DEPLOYED, this::deploy);
        stateMachine.whileState(IntakeState.RETRACTED, this::retract);
        stateMachine.whileState(IntakeState.TEST_VOLTAGE_CONTROL, () -> io.runDeployVoltage(testVoltageOutput));

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
        Angle currentTargetDeployAngle = Radians.zero();

        // Determine target angle based on state
        if (stateMachine.is(IntakeState.DEPLOYED)) {
            currentTargetDeployAngle = IntakeConstants.INTAKE_DEPLOYED_ANGLE;
        } else if (stateMachine.is(IntakeState.RETRACTED)) {
            currentTargetDeployAngle = IntakeConstants.INTAKE_RETRACTED_ANGLE;
        } else {
            return MeasuredDeployState.TRANSITION;
        }

        // Compare
        if (
            inputs.deployMotorData.positionAngle.isNear(
                currentTargetDeployAngle,
                IntakeConstants.DEPLOY_TARGET_TOLERANCE
            )
        ) {
            return stateMachine.is(IntakeState.DEPLOYED) ? MeasuredDeployState.DEPLOYED : MeasuredDeployState.RETRACTED;
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
     * @return A value from 0 to 1 representing the deploy state of the intake for visualization. Goes from 0 to 1 as the intake deploys, and from 1 to 0 as the intake retracts, with a delay to match the time it takes for the intake to deploy/retract in simulation.
     */
    private double getDeployStateForVisualization() {
        // return deployStateFilter.calculate(
        //     inputs.measuredDeployState == IntakeIO.MeasuredDeployState.DEPLOYED ? 1.0 : 0.0
        // );

        return MathUtil.inverseInterpolate(
            IntakeConstants.INTAKE_RETRACTED_ANGLE.baseUnitMagnitude(),
            IntakeConstants.INTAKE_DEPLOYED_ANGLE.baseUnitMagnitude(),
            inputs.deployMotorData.positionAngle.baseUnitMagnitude()
        );
    }

    /**
     * @return The angle to use for visualization of the intake deploy state.
     */
    public Angle getDeployAngle() {
        // return deployStateForVisualization.mut_replace(MathUtil.interpolate(
        //     IntakeConstants.INTAKE_RETRACTED_ANGLE.in(Radians),
        //     IntakeConstants.INTAKE_DEPLOYED_ANGLE.in(Radians),
        //     getDeployStateForVisualization()
        // ), Radians);

        return deployStateForVisualization.mut_replace(inputs.deployMotorData.positionAngle);
    }

    /**
     * @return A command to run the roller.
     * No requirement because also run state machine at same time (state machine does not require intake subsystem, so can run at same time as this command)
     */
    public Command runRollerCommand() {
        return Commands.run(() -> io.runRollerDutyCycle(IntakeConstants.INTAKE_RUN_SPEED));
    }

    /**
     * @return A command to stop the roller.
     */
    public Command stopRollerCommand() {
        return Commands.run(() -> io.runRollerDutyCycle(0));
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
    public void handleCalibrateDeploy() {
        runDeployDutyCycle(IntakeDeployDirection.RETRACTING, 0.175);

        if (calibrationCompleteTrigger.getAsBoolean()) {
            io.setDeployEncoderPosition(IntakeConstants.INTAKE_RETRACTED_ANGLE);
            stateMachine.scheduleStateChange(IntakeState.RETRACTED);
        }
    }

    /**
     * @return A command to do all 4 sysid routines.
     * Start with deploy retracted.
     */
    public Command sysid() {
        return new SequentialCommandGroup(
            intakeSysid.quasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(1),
            intakeSysid.quasistatic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(1),
            intakeSysid.dynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(1),
            intakeSysid.dynamic(SysIdRoutine.Direction.kReverse)
        );
    }

    /**
     * Deploys the intake.
     * Called periodically while in the {@link IntakeState#DEPLOYED} state by the state machine.
     */
    private void deploy() {
        io.setDeploySetpoint(IntakeConstants.INTAKE_DEPLOYED_ANGLE);
    }

    /**
     * Retracts the intake.
     * Called periodically while in the {@link IntakeState#RETRACTED} state by the state machine.
     */
    private void retract() {
        io.setDeploySetpoint(IntakeConstants.INTAKE_RETRACTED_ANGLE);
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

        // Visualization
        Logger.recordOutput("Intake/TestVoltOut", testVoltageOutput);
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
