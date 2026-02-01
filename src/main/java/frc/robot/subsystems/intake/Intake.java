package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANIdConstants;
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

    public enum IntakeState {
        /**
         * The intake is fully deployed.
         */
        DEPLOYED,

        /**
         * The intake is in transition between deployed and retracted.
         */
        TRANSITION_DEPLOYING,

        /**
         * The intake is in transition between retracted and deployed.
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
        .withDefaultState(new StateMachineState<>(IntakeState.RETRACTED, "Retracted"))
        .withState(new StateMachineState<>(IntakeState.DEPLOYED, "Deployed"))
        .withState(new StateMachineState<>(IntakeState.TRANSITION_DEPLOYING, "Transition Deploying"))
        .withState(new StateMachineState<>(IntakeState.TRANSITION_RETRACTING, "Transition Retracting"));

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // Alerts for disconnected motors
    private final CANIdAlert intakeDisconnectedAlert = new CANIdAlert(CANIdConstants.INTAKE_MOTOR_ID, "IntakeMotor");
    private final CANIdAlert deployDisconnectedAlert = new CANIdAlert(CANIdConstants.DEPLOY_MOTOR_ID, "DeployMotor");

    public Intake(IntakeIO io) {
        this.io = io;
    }

    // temporary
    public Command runIntake(double speed) {
        return run(() -> io.runIntake(speed));
    }

    public Command runIntake(DoubleSupplier speedSupplier) {
        return run(() -> io.runIntake(speedSupplier.getAsDouble()));
    }

    public void deploy() {
        io.deploy();
    }

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
    }
}
