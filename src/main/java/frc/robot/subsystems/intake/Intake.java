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
import frc.util.TunableValue;
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

        TEST_VOLTAGE_CONTROL,
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
        .withState(new StateMachineState<>(IntakeState.TRANSITION_RETRACTING, "TransitionRetracting"))
        .withState(new StateMachineState<>(IntakeState.TEST_VOLTAGE_CONTROL, "TestVoltage"));

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

    private final SysIdRoutine intakeSysid;

    private final MutVoltage testVoltageOutput = Volts.mutable(0.0);

    public Intake(IntakeIO io) {
        this.io = io;

        // State machine bindings
        stateMachine.whileState(IntakeState.DEPLOYED, () -> {
            // If intake is ever "undeployed" while in the deployed state, deploy again
            // if (inputs.measuredDeployState != IntakeIO.MeasuredDeployState.DEPLOYED) {
            //     stateMachine.scheduleStateChange(IntakeState.TRANSITION_DEPLOYING);
            // }

            deploy();
        });
        stateMachine.whileState(IntakeState.RETRACTED, () -> {
            retract();
        });

        stateMachine.whileState(IntakeState.TRANSITION_DEPLOYING, () -> {
            deploy();
        });
        stateMachine.whileState(IntakeState.TRANSITION_RETRACTING, () -> {
            retract();
        });

        stateMachine.whileState(IntakeState.TEST_VOLTAGE_CONTROL, () -> {
            io.runDeployVoltage(testVoltageOutput);
        });

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

    public Command runDeployDutyCycleCommand(IntakeDeployDirection direction, double speed) {
        return run(() -> runDeployDutyCycle(direction, speed));
    }

    public Command stopDeployDutyCycleCommand() {
        return run(() -> runDeployDutyCycle(IntakeDeployDirection.DEPLOYING, 0));
    }

    /**
     * @return A command that moves the intake deploy back until it hits the hard stop and then zeros the intake.
     */
    public Command calibrateIntakeDeploy() {
        /**
         * A trigger that is true when the intake deploy motor is stalled which indicates that the intake has hit the hard stop.
         */
        final Trigger calibrationCompleteTrigger = new Trigger(() ->
            inputs.deployMotorData.torqueCurrent.gte(Amps.of(22))
        ).debounce(0.25);

        return runOnce(io::removeSoftLimits)
            .andThen(
                run(() -> io.runDeployDutyCycle(-0.175))
                    .until(calibrationCompleteTrigger)
                    .andThen(
                        runOnce(() -> {
                            io.setDeployEncoderPosition(IntakeConstants.INTAKE_RETRACTED_ANGLE);
                        })
                    )
            )
            .finallyDo(io::applySoftLimits);
    }

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
     */
    public void deploy() {
        io.setDeploySetpoint(IntakeConstants.INTAKE_DEPLOYED_ANGLE);
    }

    /**
     * Retracts the intake.
     */
    public void retract() {
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
        intakeDisconnectedAlert.updateConnectionStatus(inputs.intakeMotorConnected);
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
