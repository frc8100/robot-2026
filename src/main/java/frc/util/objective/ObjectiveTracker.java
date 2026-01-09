package frc.util.objective;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotActions;
import frc.robot.RobotActions.FieldLocations;
import frc.robot.RobotActions.GlobalState;
import frc.robot.RobotActions.ScoreCoralPayload;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.swerve.Swerve.SwerveState;
import frc.util.statemachine.StateMachine;
import frc.util.statemachine.StateMachine.StateWithPayload;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks objectives and their completion status.
 */
public class ObjectiveTracker extends SubsystemBase {

    @FunctionalInterface
    public interface ObjectiveCommandSupplier {
        /**
         * Initializes the objective with the global state machine.
         * @param globalStateMachine - The global state machine of the robot.
         */
        public Command getCommand(
            StateMachine<RobotActions.GlobalState, RobotActions.GlobalPayload> globalStateMachine
        );
    }

    /**
     * The maximum number of objectives supported.
     */
    private static final int MAX_OBJECTIVES = 8;

    /**
     * The alert group name for objective alerts.
     * Should be separate from other alert groups.
     */
    private static final String ALERT_GROUP = "Objectives";

    /**
     * The status of an objective.
     */
    public enum ObjectiveStatus {
        /**
         * The objective has not been started yet.
         */
        PENDING("Pending"),

        /**
         * The objective is currently scheduled.
         */
        SCHEDULED("Scheduled"),

        /**
         * The objective is paused.
         */
        PAUSED("Paused"),

        /**
         * The objective is paused and the system should yield to the next objective.
         */
        YIELD_TO_NEXT_OBJECTIVE("Yielded"),

        /**
         * The objective has been completed
         */
        COMPLETED("Completed");

        private final String name;

        private ObjectiveStatus(String name) {
            this.name = name;
        }

        @Override
        public String toString() {
            return name;
        }
    }

    /**
     * Represents an objective to be tracked.
     */
    public static class Objective {

        /**
         * The name of the objective.
         * Should be in PascalCase for display purposes.
         */
        public final String name;

        /**
         * The priority index of the objective.
         */
        public int priorityIndex = -1;

        /**
         * The current status of the objective.
         */
        public ObjectiveStatus status = ObjectiveStatus.PENDING;

        /**
         * The command supplier for this objective.
         * ! Important: The command supplied by this must not already be wrapped or composed.
         */
        public final ObjectiveCommandSupplier commandSupplier;

        /**
         * The command associated with this objective.
         * Initialized when the objective is added to the tracker; null until then.
         */
        public Command command = null;

        public Objective(String name, ObjectiveCommandSupplier commandSupplier) {
            this.name = name;
            this.commandSupplier = commandSupplier;
        }

        /**
         * Initializes the command for this objective.
         * @param stateMachine - The global state machine of the robot.
         * @param completeObjectiveCallback - The callback to call when the objective is completed.
         */
        public void initCommand(
            StateMachine<RobotActions.GlobalState, RobotActions.GlobalPayload> stateMachine,
            Consumer<Objective> completeObjectiveCallback
        ) {
            try {
                command = commandSupplier
                    .getCommand(stateMachine)
                    .finallyDo(() -> completeObjectiveCallback.accept(this));
            } catch (IllegalArgumentException e) {
                /**
                 * Thrown in {@link CommandScheduler#registerComposedCommands(Command)} if the command is already wrapped or composed.
                 */
                e.printStackTrace();
            }
        }

        /**
         * @return The alert name for this objective.
         */
        public String getAlertName() {
            return "(" + status.toString() + ") Objective " + (priorityIndex + 1) + ": " + name;
        }
    }

    public static ObjectiveCommandSupplier switchStateAndWaitUntil(
        StateWithPayload<RobotActions.GlobalState, RobotActions.GlobalPayload> stateWithPayload,
        BooleanSupplier completeCondition
    ) {
        return globalStateMachine ->
            new Command() {
                @Override
                public void initialize() {
                    globalStateMachine.scheduleStateChange(stateWithPayload);
                }

                @Override
                public boolean isFinished() {
                    return completeCondition.getAsBoolean();
                }
            };
    }

    /**
     * "Alerts" for objectives displayed on the dashboard (not actual alerts just infos).
     * The index of the alert corresponds to the objective index.
     * This array shouldn't be modified; its contents are modified.
     * If there are more than {@link #MAX_OBJECTIVES} objectives, the extra ones won't have alerts.
     */
    private final Alert[] objectiveAlerts = new Alert[MAX_OBJECTIVES];

    /**
     * The list of objectives being tracked.
     */
    private final LinkedList<Objective> listOfObjectives = new LinkedList<>();

    private final RobotActions robotActions;

    private Objective currentlyScheduledObjective = null;

    /**
     * Constructs an ObjectiveTracker given the robot actions.
     * @param robotActions - The robot actions instance.
     */
    public ObjectiveTracker(RobotActions robotActions) {
        this.robotActions = robotActions;

        // Initialize objective alerts
        for (int i = 0; i < MAX_OBJECTIVES; i++) {
            objectiveAlerts[i] = new Alert(ALERT_GROUP, "Objective " + (i + 1) + ": None", Alert.AlertType.kInfo);

            // Set initial alert to true so it shows up on the dashboard
            objectiveAlerts[i].set(true);
        }

        // Test usage
        // addObjective(
        //     new Objective(
        //         "TestIntake",
        //         switchStateAndWaitUntil(
        //             new StateWithPayload<>(
        //                 GlobalState.INTAKE_CORAL_FROM_STATION,
        //                 new RobotActions.IntakeCoralPayload(FieldLocations.CORAL_STATION_1)
        //             ),
        //             () -> robotActions.swerveSubsystem.stateMachine.is(SwerveState.DRIVE_TO_POSE_AT_TARGET)
        //         )
        //     )
        // );
        // addObjective(
        //     new Objective(
        //         "TestScore",
        //         switchStateAndWaitUntil(
        //             new StateWithPayload<>(
        //                 GlobalState.SCORE_CORAL,
        //                 new RobotActions.ScoreCoralPayload(FieldLocations.REEF_1L, SuperstructureConstants.Level.L4)
        //             ),
        //             () -> robotActions.swerveSubsystem.stateMachine.is(SwerveState.DRIVE_TO_POSE_AT_TARGET)
        //         )
        //     )
        // );

        SmartDashboard.putData("ScheduleNextObjective", Commands.runOnce(this::scheduleNextObjective));
    }

    /**
     * Updates the objective alerts and their priority indices.
     */
    private void updateObjectiveAlertsAndIndices() {
        for (int i = 0; i < listOfObjectives.size(); i++) {
            Objective obj = listOfObjectives.get(i);
            obj.priorityIndex = i;

            if (i < MAX_OBJECTIVES) {
                objectiveAlerts[i].setText(obj.getAlertName());
            }
        }

        // Set rest of alerts to none
        for (int i = listOfObjectives.size(); i < MAX_OBJECTIVES; i++) {
            objectiveAlerts[i].setText("Objective " + (i + 1) + ": None");
        }
    }

    /**
     * Adds an objective to be tracked.
     * @param objective - The objective to add.
     * @param index - The index at which to add the objective. If -1, adds to the end.
     */
    public void addObjective(Objective objective, int index) {
        // Check index bounds
        if (index < 0 || index > listOfObjectives.size()) {
            listOfObjectives.add(objective);
            objective.priorityIndex = listOfObjectives.size() - 1;
        } else {
            listOfObjectives.add(index, objective);
            objective.priorityIndex = index;
        }

        updateObjectiveAlertsAndIndices();

        // Initialize the objective
        objective.initCommand(robotActions.globalStateMachine, this::completeObjective);
    }

    /**
     * Adds an objective to be tracked at the end of the list.
     * @param objective - The objective to add.
     */
    public void addObjective(Objective objective) {
        addObjective(objective, -1);
    }

    /**
     * Marks an objective as completed and removes it from tracking.
     * @param objective - The objective to complete.
     */
    public void completeObjective(Objective objective) {
        objective.status = ObjectiveStatus.COMPLETED;
        listOfObjectives.remove(objective);

        updateObjectiveAlertsAndIndices();
        scheduleNextObjective();
    }

    /**
     * Schedules the next pending objective, if one exists.
     */
    public void scheduleNextObjective() {
        Objective objectiveToScheduleNext = getNextScheduledObjective();

        if (objectiveToScheduleNext == null) {
            return;
        }

        // Clear currently scheduled objective
        currentlyScheduledObjective = null;

        currentlyScheduledObjective = objectiveToScheduleNext;

        if (currentlyScheduledObjective.command != null) {
            objectiveToScheduleNext.status = ObjectiveStatus.SCHEDULED;
            updateObjectiveAlertsAndIndices();

            try {
                CommandScheduler.getInstance().schedule(currentlyScheduledObjective.command);
            } catch (IllegalArgumentException e) {
                // Command is composed
                e.printStackTrace();
            }
        }
    }

    /**
     * @return The next objective that is pending, or null if none exist.
     */
    public Objective getNextScheduledObjective() {
        for (int i = 0; i < listOfObjectives.size(); i++) {
            Objective objective = listOfObjectives.get(i);

            if (objective.status == ObjectiveStatus.PENDING) {
                return objective;
            }
        }

        return null;
    }

    /**
     * Periodically checks and schedules the next objective if none is currently scheduled.
     */
    @Override
    public void periodic() {
        // Only run if enabled
        if (DriverStation.isDisabled()) {
            return;
        }

        if (currentlyScheduledObjective == null) {
            scheduleNextObjective();
        }
    }
}
