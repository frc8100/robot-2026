package frc.util.objective;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotActions;
import frc.util.objective.ObjectiveIO.ShiftMode;
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

    // Inspired by https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/567

    /**
     * The time that fuel takes after it has entered the hub to be counted by the sensors.
     */
    public static final Time TIME_FOR_HUB_TO_PROCESS_AFTER_SCORE_MIN = Seconds.of(1.0);
    public static final Time TIME_FOR_HUB_TO_PROCESS_AFTER_SCORE_MAX = Seconds.of(2.0);

    /**
     * The time after a shift ends where fuel can still score.
     * Specified by game manual.
     */
    public static final Time TIME_AFTER_SHIFT_ENDS_WHERE_HUB_STILL_COUNTS = Seconds.of(3.0);

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
    private final List<Objective> listOfObjectives = new LinkedList<>();

    private final RobotActions robotActions;

    private Objective currentlyScheduledObjective = null;

    private final ObjectiveIO io;
    private final ObjectiveIOInputsAutoLogged inputs = new ObjectiveIOInputsAutoLogged();

    /**
     * Constructs an ObjectiveTracker given the robot actions.
     * @param robotActions - The robot actions instance.
     * @param objectiveIO - The objective IO instance.
     */
    public ObjectiveTracker(RobotActions robotActions, ObjectiveIO objectiveIO) {
        this.robotActions = robotActions;
        this.io = objectiveIO;

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
     * Determines if the robot is able to shoot and score based on the time of flight.
     * @param timeOfFlight - The time it takes for the projectile to reach the target.
     * @return The time remaining where the robot can shoot and score
     * If this is positive, the robot can shoot and score, and it will count down for the time that the robot can score.
     * If this is negative, the robot cannot shoot and score, and it will count up until the robot can score again.
     */
    public double isAbleToShootAndScore(Time timeOfFlight) {
        // Always able to shoot during auto
        if (DriverStation.isAutonomous()) {
            return inputs.timeUntilSwitch.in(Seconds);
        }

        // Determine if the current shift is active
        boolean isCurrentShiftActive = false;
        // boolean isNextShiftActive = false;
        var currentAlliance = DriverStation.getAlliance();

        // If the alliance is not determined, allow shooting
        if (currentAlliance.isEmpty()) {
            return inputs.timeUntilSwitch.in(Seconds);
        }

        // double flight = timeOfFlight.in(Seconds);
        // double process = TIME_FOR_HUB_TO_PROCESS_AFTER_SCORE.in(Seconds);
        // double countExtension = TIME_AFTER_SHIFT_ENDS_WHERE_HUB_STILL_COUNTS.in(Seconds);

        // double launchDelay = flight + process;

        double baseTimeWhenFuelScores =
            // Current time
            (inputs.currentShiftMode.startTimeSeconds + inputs.timeElapsedFromLastSwitch.in(Seconds)) +
            // (inputs.timeElapsedFromLastSwitch.in(Seconds)) +
            // Time for fuel to go in
            (timeOfFlight.in(Seconds));
        double timeWhenFuelScores = baseTimeWhenFuelScores;

        // Determine the shift that the fuel lands on
        // for (ShiftMode shiftMode : ObjectiveIO.switches) {
        for (int i = 1; i < ObjectiveIO.switches.length; i++) {
            ShiftMode shiftMode = ObjectiveIO.switches[i];

            boolean isActive = shiftMode.active.toActiveHub(inputs.initialActiveHub).isActive(currentAlliance.get());

            timeWhenFuelScores =
                baseTimeWhenFuelScores +
                (isActive
                        ? ObjectiveTracker.TIME_FOR_HUB_TO_PROCESS_AFTER_SCORE_MAX.in(Seconds)
                        : ObjectiveTracker.TIME_FOR_HUB_TO_PROCESS_AFTER_SCORE_MIN.in(Seconds));

            if (
                timeWhenFuelScores >= shiftMode.startTimeSeconds &&
                (timeWhenFuelScores <
                    (isActive
                            ? shiftMode.endTimeSeconds +
                            ObjectiveTracker.TIME_AFTER_SHIFT_ENDS_WHERE_HUB_STILL_COUNTS.in(Seconds)
                            : shiftMode.endTimeSeconds))
            ) {
                ShiftMode nextShiftMode = shiftMode;
                boolean isNextShiftActive = true;

                if (i + 1 < ObjectiveIO.switches.length) {
                    nextShiftMode = ObjectiveIO.switches[i + 1];
                    isNextShiftActive = nextShiftMode.active
                        .toActiveHub(inputs.initialActiveHub)
                        .isActive(currentAlliance.get());
                }

                // test
                Logger.recordOutput("ObjectiveTracker/TimeWhenFuelScores", timeWhenFuelScores);
                Logger.recordOutput("ObjectiveTracker/CurrentFuelShiftMode", shiftMode.toString());
                Logger.recordOutput("ObjectiveTracker/NextFuelShiftMode", nextShiftMode.toString());

                if (isActive) {
                    if (isNextShiftActive) {
                        if (nextShiftMode == ObjectiveIO.ShiftMode.ENDGAME) {
                            return (
                                nextShiftMode.endTimeSeconds -
                                // Current time
                                (inputs.currentShiftMode.startTimeSeconds +
                                    inputs.timeElapsedFromLastSwitch.in(Seconds))
                            );
                        }

                        // Both current and next shifts are active
                        return (
                            nextShiftMode.endTimeSeconds +
                            ObjectiveTracker.TIME_AFTER_SHIFT_ENDS_WHERE_HUB_STILL_COUNTS.in(Seconds) -
                            timeWhenFuelScores
                        );
                    }

                    // Current is active, next inactive
                    return (
                        shiftMode.endTimeSeconds +
                        ObjectiveTracker.TIME_AFTER_SHIFT_ENDS_WHERE_HUB_STILL_COUNTS.in(Seconds) -
                        timeWhenFuelScores
                    );
                }

                // Current is inactive, next active; return negative which "counts up" to 0
                return -(nextShiftMode.startTimeSeconds - timeWhenFuelScores);
            }
        }

        return inputs.timeUntilSwitch.in(Seconds);
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
        // Update the IO inputs
        io.updateInputs(inputs);
        Logger.processInputs("Objective", inputs);

        double canShootRemaining = isAbleToShootAndScore(Seconds.one());
        boolean canShoot = canShootRemaining > 0;

        Logger.recordOutput("ObjectiveTracker/TimeRemainingCanShoot", String.format("%.1f", canShootRemaining));

        Logger.recordOutput("ObjectiveTracker/CurrentShift", inputs.currentShiftMode.toString());
        Logger.recordOutput("ObjectiveTracker/ActiveAllianceColor", inputs.activeHub.color);

        // Only run if enabled
        if (DriverStation.isDisabled()) {
            return;
        }

        if (currentlyScheduledObjective == null) {
            scheduleNextObjective();
        }
    }
}
